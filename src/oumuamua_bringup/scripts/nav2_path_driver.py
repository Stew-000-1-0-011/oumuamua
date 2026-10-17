#!/usr/bin/env python3
"""path_follower と同じ経路ファイルを Nav2 に渡すノード (navigation:=nav2 のとき、path_follower の代わりに立てる)。

path_follower と同じ名前 (`path_follower`) で同じサービスを出すので、走り出させ方は変わらない:

- `~/start/<名前>` / `~/start` (std_srvs/Trigger): その経路で Nav2 に走らせる
- `~/stop` (std_srvs/Trigger): Nav2 のゴールを取り消す

`mode` で Nav2 への渡し方を選ぶ:

- `follow_path`  : 経路ファイルの点をそのまま nav_msgs/Path にして、controller_server の FollowPath に渡す
                   (経路を引くのは同じで、追従のしかただけが違う)
- `through_poses`: 経路を `waypoint_spacing` ごとに間引いた姿勢を、bt_navigator の NavigateThroughPoses に渡す
                   (経路も Nav2 が地図から引き直す)

Nav2 は時刻つきの目標を出さないので、`reference_topic` (TrackingReference) には、
ゴールに着いた (アクションが成功した) ときに終点を 1 回だけ出す。oumuamua_bench はこれで区間の終わりを知る。
"""

import math
from pathlib import Path as FsPath

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from holonomic_tracker.msg import TrackingReference
from nav2_msgs.action import FollowPath, NavigateThroughPoses
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


def load_path(path_file):
    rows = []
    for line in FsPath(path_file).read_text().splitlines():
        if not line or line.startswith('#') or line[0].isalpha():
            continue
        rows.append([float(v) for v in line.split(',')])
    return rows


class Nav2PathDriver(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.frame_id = self.declare_parameter('frame_id', 'field').value
        path_dir = self.declare_parameter('path_dir', '').value
        names = self.declare_parameter('paths', ['start_to_shelf']).value
        self.mode = self.declare_parameter('mode', 'follow_path').value
        self.spacing = self.declare_parameter('waypoint_spacing', 0.5).value
        self.controller_id = self.declare_parameter('controller_id', 'FollowPath').value
        self.goal_checker_id = self.declare_parameter('goal_checker_id', 'general_goal_checker').value
        reference_topic = self.declare_parameter('reference_topic', '/tracker_node/reference').value

        self.paths = {name: load_path(FsPath(path_dir) / f'{name}.csv') for name in names}
        self.ref_pub = self.create_publisher(TrackingReference, reference_topic, 10)
        self.path_pub = self.create_publisher(Path, '~/path', rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.follow = ActionClient(self, FollowPath, 'follow_path')
        self.through = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.goal_handle = None
        for i, name in enumerate(names):
            self.create_service(Trigger, f'~/start/{name}', lambda req, res, n=name: self.on_start(n, res))
            if i == 0:
                self.create_service(Trigger, '~/start', lambda req, res, n=name: self.on_start(n, res))
        self.create_service(Trigger, '~/stop', lambda req, res: self.on_stop(res))
        self.get_logger().info(f'mode {self.mode}, paths: {", ".join(names)}')

    def pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.frame_id
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(0.5 * yaw)
        p.pose.orientation.w = math.cos(0.5 * yaw)
        return p

    def on_start(self, name, res):
        rows = self.paths[name]
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.pose(*r) for r in rows]
        self.path_pub.publish(path)

        if self.mode == 'follow_path':
            if not self.follow.wait_for_server(timeout_sec=1.0):
                res.success, res.message = False, 'follow_path action is not available'
                return res
            goal = FollowPath.Goal()
            goal.path = path
            goal.controller_id = self.controller_id
            goal.goal_checker_id = self.goal_checker_id
            future = self.follow.send_goal_async(goal)
        else:
            if not self.through.wait_for_server(timeout_sec=1.0):
                res.success, res.message = False, 'navigate_through_poses action is not available'
                return res
            # 道のり waypoint_spacing ごとに間引く (終点は必ず入れる)
            poses = []
            travelled = 0.0
            for prev, cur in zip(rows, rows[1:]):
                travelled += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
                if travelled >= self.spacing:
                    poses.append(self.pose(*cur))
                    travelled = 0.0
            if not poses or poses[-1].pose.position.x != rows[-1][0] or poses[-1].pose.position.y != rows[-1][1]:
                poses.append(self.pose(*rows[-1]))
            goal = NavigateThroughPoses.Goal()
            goal.poses.header.frame_id = self.frame_id
            goal.poses.header.stamp = self.get_clock().now().to_msg()
            goal.poses.goals = poses
            future = self.through.send_goal_async(goal)
        future.add_done_callback(lambda f, n=name: self.on_accepted(n, f))
        res.success, res.message = True, f'sent {name} to Nav2 ({self.mode})'
        return res

    def on_accepted(self, name, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'{name}: goal rejected')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f, n=name: self.on_result(n, f))

    def on_result(self, name, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'{name}: Nav2 finished with status {status}')
            return
        x, y, yaw = self.paths[name][-1]
        ref = TrackingReference()
        ref.header.frame_id = self.frame_id
        ref.header.stamp = self.get_clock().now().to_msg()
        ref.x, ref.y, ref.yaw = x, y, yaw
        self.ref_pub.publish(ref)
        self.get_logger().info(f'{name}: reached the goal')

    def on_stop(self, res):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        res.success, res.message = True, 'cancelled'
        return res


def main():
    rclpy.init()
    rclpy.spin(Nav2PathDriver())


if __name__ == '__main__':
    main()
