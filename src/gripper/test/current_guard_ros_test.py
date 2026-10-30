"""Exercise both mirrored lifts with synthetic ESC feedback; no hardware needed."""
import os
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from gripper.action import MoveLift
from gripper.msg import GripperState
from gripper.srv import SetLiftPosition
from robomas_plugins.msg import Frame, RobomasFrame, RobomasTarget
from std_srvs.srv import SetBool

EXECUTABLE, CONFIG = sys.argv[1:3]
sys.argv = sys.argv[:1]


class CurrentProtectionTest(unittest.TestCase):
    def test_mirrored_lifts(self):
        for name, motor, sign in [("gripper0", 2, 1), ("gripper1", 1, -1)]:
            for coincident in (False, True):
                with self.subTest(lift=name, coincident_goal=coincident):
                    self.exercise(name, motor, sign, coincident)

    def exercise(self, name, motor, sign, coincident):
        # Isolate this fake motor from other ROS tests / robot topics.
        os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
        rclpy.init()
        node = rclpy.create_node("current_guard_test")
        self.node = node
        self.state = None
        self.target = None
        self.modes = []
        self.angle = 0
        self.raw_current = 0
        self.feedback_on = True
        self.motor = motor
        self.pub = node.create_publisher(Frame, "/robomas_can_rx", 100)
        subscriptions = [
            node.create_subscription(GripperState, f"/{name}/state", lambda m: setattr(self, "state", m), 10),
            node.create_subscription(RobomasTarget, f"/robomas_target{motor}", lambda m: setattr(self, "target", m.target), 10),
            node.create_subscription(RobomasFrame, "/robomas_frame", lambda m: self.modes.append(m.mode) if m.motor == motor - 1 else None, 10),
        ]
        position = node.create_client(SetLiftPosition, f"/{name}/set_lift_position")
        enable = node.create_client(SetBool, f"/{name}/enable_lift")
        action = ActionClient(node, MoveLift, f"/{name}/move_lift")
        with tempfile.TemporaryFile(mode="w+") as log:
            process = subprocess.Popen([
                EXECUTABLE, "--ros-args", "--params-file", CONFIG, "-r", f"__node:={name}",
                "-p", "lift.config_repeat:=1", "-p", "lift.overcurrent_threshold:=8.0",
                "-p", "lift.overcurrent_duration:=0.2",
            ], stdout=log, stderr=subprocess.STDOUT)
            try:
                self.assertTrue(position.wait_for_service(timeout_sec=10.0))
                self.wait(lambda: self.state is not None and self.state.lift_feedback)
                # Simulate moving from zero to ~0.12 m, with no high current yet.
                for _ in range(188):
                    self.angle = (self.angle + sign * 2000) % 8192
                    self.tick()
                self.wait(lambda: self.state.lift_height > 0.11)
                self.pump(0.15)
                target_height = self.state.lift_height if coincident else 0.18
                self.assertTrue(action.wait_for_server(timeout_sec=5.0))
                initial = self.future(action.send_goal_async(MoveLift.Goal(height=target_height)))
                self.assertTrue(initial.accepted)
                self.wait(lambda: abs(self.state.lift_command - target_height) < 1e-6)
                if coincident:
                    self.assertAlmostEqual(self.state.lift_command, self.state.lift_height, places=6)
                # A short spike must not stop the lift.
                self.raw_current = sign * 9000
                self.pump(0.08)
                self.raw_current = 0
                self.pump(0.15)
                self.assertFalse(self.state.lift_upper_limited)
                self.assertTrue(action.wait_for_server(timeout_sec=5.0))
                active_goal = self.future(action.send_goal_async(MoveLift.Goal(height=target_height)))
                self.assertTrue(active_goal.accepted)
                action_result = active_goal.get_result_async()
                # Stalling on the upper stop for long enough trips and recenters.
                self.raw_current = sign * 9000
                modes_before = len(self.modes)
                self.wait(lambda: self.state.lift_upper_limited)
                self.raw_current = 0
                self.pump(0.15)
                self.assertFalse(self.state.lift_moving)
                self.assertFalse(self.state.lift_arrived)
                self.assertAlmostEqual(self.state.lift_command, self.state.lift_height, places=6)
                self.assertAlmostEqual(self.target, 0.0, places=3)
                self.assertIn(2, self.modes[modes_before:])  # PI / board origin reset, holding enabled
                result = self.future(action_result)
                self.assertEqual(result.status, 6)  # STATUS_ABORTED
                self.assertFalse(result.result.reached)
                self.assertIn("overcurrent", result.result.message)
                self.assertFalse(self.call(position, SetLiftPosition.Request(index=2)).success)
                self.assertTrue(action.wait_for_server(timeout_sec=5.0))
                rejected = self.future(action.send_goal_async(MoveLift.Goal(height=0.18)))
                self.assertFalse(rejected.accepted)
                # An explicit retreat must work with the new board origin on both lifts.
                self.assertTrue(self.call(position, SetLiftPosition.Request(index=1)).success)
                self.wait(lambda: self.target is not None and sign * self.target < -1.0)
                self.assertTrue(self.state.lift_upper_limited)
                # Enable toggles cannot clear the limit and retry the blocked goal.
                self.assertTrue(self.call(enable, SetBool.Request(data=False)).success)
                self.assertTrue(self.call(enable, SetBool.Request(data=True)).success)
                self.assertFalse(self.call(position, SetLiftPosition.Request(index=2)).success)
                # Stale feedback cannot be used to reset the physical board origin.
                self.feedback_on = False
                self.pump(0.3)
                self.assertFalse(self.state.lift_feedback)
                self.pump(0.05)
                mode_count = len(self.modes)
                self.assertFalse(self.call(enable, SetBool.Request(data=True)).success)
                self.pump(0.1)
                self.assertEqual(len(self.modes), mode_count)
                self.feedback_on = True
                self.wait(lambda: self.state.lift_feedback)
                self.assertFalse(self.call(position, SetLiftPosition.Request(index=2)).success)
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                log.seek(0)
                print(log.read())
                action.destroy()
                node.destroy_node()
                rclpy.shutdown()

    def tick(self):
        frame = Frame()
        frame.id = 0x200 + self.motor
        frame.dlc = 8
        current = self.raw_current & 0xffff
        frame.data = [self.angle >> 8, self.angle & 255, 0, 0, current >> 8, current & 255, 0, 0]
        if self.feedback_on:
            self.pub.publish(frame)
        rclpy.spin_once(self.node, timeout_sec=0.005)
        time.sleep(0.005)

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.tick()

    def wait(self, predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), end, "timed out waiting for ROS state")
            self.tick()

    def future(self, future):
        self.wait(future.done)
        return future.result()

    def call(self, client, request):
        return self.future(client.call_async(request))


if __name__ == "__main__":
    unittest.main()
