"""Exercise the real task node against synthetic ROS actuator/action servers.

Called by CTest with task_commander_node and the source bringup config directory.
"""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from gripper.action import MoveLift
from gripper.msg import GripperState
from nav_msgs.msg import Odometry
from path_follower.action import FollowPath
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from task_commander.action import RunTask
import yaml
import math

BINARY, CONFIG = sys.argv[1], Path(sys.argv[2])
sys.argv = [sys.argv[0]]


def wait(future, timeout=8):
    end = time.monotonic()+timeout
    while not future.done() and time.monotonic()<end:
        time.sleep(.01)
    if not future.done():
        raise TimeoutError('ROS response timed out')
    return future.result()


class Robot(Node):
    def __init__(self, start):
        super().__init__('pickup_test_robot')
        self.group = ReentrantCallbackGroup()
        self.start = start
        self.pose = list(start)
        self.heights = [.18, .18]
        self.suction = [False, False]
        self.mode = ''
        self.feedback = True
        self.moves = self.lifts = 0
        self.lift_canceled = False
        self.state = ''
        self.odom = self.create_publisher(Odometry, '/state_estimator/odom', 10)
        self.grippers = [self.create_publisher(GripperState, f'/gripper{i}/state', 10) for i in range(2)]
        self.suction_services = []
        self.actions = []
        for i in range(2):
            self.suction_services.append(self.create_service(SetBool, f'/gripper{i}/set_suction',
                lambda req, res, i=i: self.set_suction(i, req, res), callback_group=self.group))
            self.actions.append(ActionServer(self, MoveLift, f'/gripper{i}/move_lift',
                lambda goal, i=i: self.lift(i, goal),
                cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group))
        self.actions.append(ActionServer(self, FollowPath, '/path_follower/follow', self.move,
                                        cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/task_commander/state', lambda m: setattr(self, 'state', m.data), qos)
        self.create_timer(.02, self.publish, callback_group=self.group)
        self.tasks = ActionClient(self, RunTask, '/task_commander/run', callback_group=self.group)
        self.reset = self.create_client(Trigger, '/task_commander/reset', callback_group=self.group)

    def set_suction(self, i, request, response):
        self.suction[i] = request.data
        if self.mode == 'suction_response_lost':
            time.sleep(2.3) # Actuator applied the request; the task's 2 s service wait expires.
        response.success = True
        return response

    def lift(self, i, goal):
        self.lifts += 1
        result = MoveLift.Result()
        if self.mode == 'cancel_lift':
            end = time.monotonic()+3
            while not goal.is_cancel_requested and time.monotonic()<end:
                time.sleep(.01)
            self.lift_canceled = goal.is_cancel_requested
            if self.lift_canceled:
                goal.canceled()
            else:
                goal.abort()
            result.reached = False
            result.height = self.heights[i]
            return result
        if self.mode == 'carry_failure' and self.suction[i] and goal.request.height == .18:
            result.reached = False
            result.height = self.heights[i]
            result.message = 'synthetic carry failure'
            goal.abort()
        else:
            self.heights[i] = goal.request.height
            self.publish()
            result.reached = True
            result.height = self.heights[i]
            goal.succeed()
        return result

    def move(self, goal):
        self.moves += 1
        result = FollowPath.Result()
        if self.mode in ('drop_lift_feedback', 'stale_during_motion'):
            if self.mode == 'drop_lift_feedback':
                self.feedback = False
            else:
                self.mode = 'stale_odom'
            self.publish()
            end = time.monotonic()+.5
            while not goal.is_cancel_requested and time.monotonic()<end:
                time.sleep(.01)
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
            result.success = False
            return result
        if self.mode == 'transit_failure' or (self.mode == 'retreat_failure' and self.suction[0]):
            result.success = False
            result.message = 'synthetic transit failure'
            goal.abort()
        else:
            self.pose = [goal.request.x[-1], goal.request.y[-1], goal.request.yaw[-1]]
            self.publish()
            result.success = True
            goal.succeed()
        return result

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        if self.mode == 'stale_odom':
            stamp.sec -= 1
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = 'field'
        message.pose.pose.position.x, message.pose.pose.position.y = self.pose[:2]
        message.pose.pose.orientation.z = math.sin(self.pose[2]/2)
        message.pose.pose.orientation.w = math.cos(self.pose[2]/2)
        self.odom.publish(message)
        for i, publisher in enumerate(self.grippers):
            message = GripperState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.lift_height = self.heights[i]
            message.lift_goal = self.heights[i]
            message.lift_arrived = True
            message.lift_moving = False
            message.lift_feedback = self.feedback
            message.suction_on = self.suction[i]
            publisher.publish(message)


class PickupRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        spec = importlib.util.spec_from_file_location('params', CONFIG.parent/'launch/pickup_parameters.py')
        params = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(params)
        overrides = params.pickup_parameters(CONFIG/'task_commander.yaml', CONFIG/'gripper.yaml',
                                             CONFIG/'footprints/raised.yaml', CONFIG/'footprints/lowered.yaml')
        overrides.update({'pick.grip_time': .02, 'lift.timeout': 2.0, 'pick.motion_timeout': 2.0})
        override_file = Path(cls.temp.name)/'overrides.yaml'
        override_file.write_text(yaml.safe_dump({'task_commander': {'ros__parameters': overrides}}))
        field = yaml.safe_load((CONFIG/'field.yaml').read_text())['/**']['ros__parameters']['field']
        rclpy.init()
        cls.robot = Robot(field['start_pose'])
        cls.executor = MultiThreadedExecutor(num_threads=6)
        cls.executor.add_node(cls.robot)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        cls.log = open(Path(cls.temp.name)/'node.log', 'w+')
        cls.process = subprocess.Popen([BINARY, '--ros-args', '--params-file', str(CONFIG/'field.yaml'),
                                       '--params-file', str(CONFIG/'task_commander.yaml'), '--params-file', str(override_file)],
                                      stdout=cls.log, stderr=subprocess.STDOUT)
        if not cls.robot.tasks.wait_for_server(timeout_sec=10):
            cls.log.flush(); cls.log.seek(0)
            raise RuntimeError('task server did not start:\n'+cls.log.read())

    @classmethod
    def tearDownClass(cls):
        cls.process.terminate()
        try: cls.process.wait(timeout=5)
        except subprocess.TimeoutExpired: cls.process.kill(); cls.process.wait()
        cls.executor.shutdown()
        cls.robot.destroy_node()
        rclpy.shutdown()
        cls.thread.join(timeout=2)
        cls.log.close()
        cls.temp.cleanup()

    def setUp(self):
        self.robot.mode = ''
        self.robot.feedback = True
        self.robot.pose = list(self.robot.start)
        self.robot.heights = [.18, .18]
        self.robot.suction = [False, False] # explicit physical reconciliation between cases
        self.assertTrue(self.robot.reset.wait_for_service(timeout_sec=3))
        end = time.monotonic()+3
        while not wait(self.robot.reset.call_async(Trigger.Request())).success:
            self.assertLess(time.monotonic(), end)
            time.sleep(.03)
        time.sleep(.1)
        self.robot.moves = self.robot.lifts = 0
        self.robot.lift_canceled = False

    def pick(self, gripper=0, box=4):
        goal = RunTask.Goal(); goal.type = 0; goal.gripper = gripper; goal.block = box
        end = time.monotonic()+1
        while True:
            handle = wait(self.robot.tasks.send_goal_async(goal))
            if handle.accepted or time.monotonic()>=end:
                break
            time.sleep(.02) # result publication may precede the node's busy flag release
        self.assertTrue(handle.accepted)
        return wait(handle.get_result_async(), 15).result

    def wait_state(self, text):
        end = time.monotonic()+2
        while text not in self.robot.state and time.monotonic()<end:
            time.sleep(.01)
        self.assertIn(text, self.robot.state)

    def test_success_and_holding_rejection(self):
        result = self.pick()
        self.assertTrue(result.success, result.message)
        self.wait_state('gripper0=yes')
        self.assertIn('picked blocks: 4', self.robot.state)
        result = self.pick(1, 3)
        self.assertTrue(result.success, result.message)
        self.wait_state('gripper1=yes')
        result = self.pick()
        self.assertFalse(result.success)

    def test_failure_before_suction_is_retryable(self):
        self.robot.mode = 'transit_failure'
        self.assertFalse(self.pick().success)
        self.assertFalse(self.robot.suction[0])
        self.assertNotIn('picked blocks: 4', self.robot.state)
        self.robot.mode = ''
        self.assertTrue(self.pick().success)

    def test_carry_failure_interlocks_auto_until_reconciled(self):
        self.robot.mode = 'carry_failure'
        result = self.pick()
        self.assertFalse(result.success)
        self.assertIn('uncertain', result.message)
        self.assertIn('gripper0=yes', self.robot.state)
        self.assertIn('picked blocks: 4', self.robot.state)
        self.wait_state('uncertain grippers: gripper0')
        goal = RunTask.Goal(); goal.type = 2
        self.assertFalse(wait(self.robot.tasks.send_goal_async(goal)).accepted)

    def test_stale_source_odometry_never_moves_actuators(self):
        self.robot.mode = 'stale_odom'
        time.sleep(.1)
        self.assertFalse(self.pick().success)
        self.assertEqual(self.robot.lifts, 0)
        self.assertEqual(self.robot.moves, 0)

    def test_feedback_loss_cancels_transit_before_suction(self):
        self.robot.mode = 'drop_lift_feedback'
        result = self.pick()
        self.assertFalse(result.success)
        self.assertFalse(self.robot.suction[0])
        self.assertNotIn('picked blocks: 4', self.robot.state)

    def test_stale_odometry_during_transit_cancels_before_suction(self):
        self.robot.mode = 'stale_during_motion'
        self.assertFalse(self.pick().success)
        self.assertFalse(self.robot.suction[0])

    def test_suction_applied_without_response_interlocks_load(self):
        self.robot.mode = 'suction_response_lost'
        self.assertFalse(self.pick().success)
        self.assertTrue(self.robot.suction[0])
        self.wait_state('uncertain grippers: gripper0')
        self.assertEqual(self.robot.moves, 2) # transit and approach; no retreat
        self.assertAlmostEqual(self.robot.heights[0], 0.) # no carry after missing response

    def test_retreat_failure_keeps_confirmed_load(self):
        self.robot.mode = 'retreat_failure'
        self.assertFalse(self.pick().success)
        self.wait_state('gripper0=yes')
        self.assertIn('picked blocks: 4', self.robot.state)
        self.assertNotIn('uncertain grippers: gripper0', self.robot.state)
        self.assertAlmostEqual(self.robot.heights[0], .18)

    def test_cancellation_stops_active_lift_before_transit(self):
        self.robot.mode = 'cancel_lift'
        goal = RunTask.Goal(); goal.type = 0; goal.gripper = 0; goal.block = 4
        handle = wait(self.robot.tasks.send_goal_async(goal))
        self.assertTrue(handle.accepted)
        end = time.monotonic()+3
        while not self.robot.lifts and time.monotonic()<end:
            time.sleep(.01)
        self.assertEqual(self.robot.lifts, 1)
        wait(handle.cancel_goal_async())
        self.assertFalse(wait(handle.get_result_async()).result.success)
        self.assertTrue(self.robot.lift_canceled)
        self.assertEqual(self.robot.moves, 0)
        self.assertFalse(self.robot.suction[0])


if __name__ == '__main__':
    unittest.main()
