#!/usr/bin/env python3
"""ロボットを止める (速度 0)。scripts/stop.sh から呼ぶ。

経路の目標 (path_follower) と追従 (tracker_node) を止め、その間 /cmd_vel に速度 0 を流し続ける。
--free なら各車輪の出力も切る。ros2 CLI を何度も起動すると数秒かかるので、1 つのノードでまとめて行う。
"""
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


def main():
    free = '--free' in sys.argv[1:]
    rclpy.init()
    node = Node('stop_robot')
    pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)

    def zero():
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    calls = [(node.create_client(Trigger, '/path_follower/stop'), Trigger.Request())]
    disable = SetBool.Request(data=False)
    calls.append((node.create_client(SetBool, '/tracker_node/enable'), disable))
    if free:
        calls += [(node.create_client(SetBool, f'/wheel{i}/enable'), disable) for i in range(3)]

    pending = {}
    results = {}
    start = time.monotonic()
    # 速度 0 を 50 Hz で流しながら、各サービスが見つかり次第呼ぶ。全部終わってからさらに 0.3 s 流す
    done_at = None
    while rclpy.ok():
        zero()
        now = time.monotonic()
        for client, request in calls:
            name = client.srv_name
            if name in results or name in pending:
                continue
            if client.service_is_ready():
                pending[name] = client.call_async(request)
        for name, future in list(pending.items()):
            if future.done():
                results[name] = future.result() is not None and future.result().success
                del pending[name]
        if done_at is None and len(results) == len(calls):
            done_at = now
        if done_at is not None and now - done_at > 0.3:
            break
        if now - start > 3.0:
            break
        rclpy.spin_once(node, timeout_sec=0.02)

    for client, _ in calls:
        state = results.get(client.srv_name)
        print(f'{client.srv_name}: ' + ('ok' if state else 'failed' if state is not None else 'no service'))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
