#!/usr/bin/env python3
"""吸引の状態を CSV に記録する (受信だけ)。scripts/record_suction.sh から呼ぶ。

/suction{0,1}/status (mini_shirasu_node)、/suction{0,1}/target_current と /gripper{0,1}/state (gripper_node) を
受けたまま 1 行ずつ書く。終わったら、目標電流が 0 でなかった間の電流を左右ごとにまとめて出す。
"""
import argparse
import csv
import statistics
import time

import rclpy
from gripper.msg import GripperState
from mini_shirasu_ros.msg import Status
from std_msgs.msg import Float64

SIDES = (0, 1)
NAMES = {0: '左 (suction0)', 1: '右 (suction1)'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    parser.add_argument('--duration', type=float, default=0.0, help='[s] 0 なら Ctrl+C まで')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('suction_recorder')
    f = open(args.output, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['t', 'wall', 'side', 'kind', 'mode', 'current', 'velocity', 'vdc',
                'sat_up', 'sat_lo', 'vdc_low', 'temp_raw', 'value'])
    t0 = time.time()
    target = {i: 0.0 for i in SIDES}
    # 目標電流が 0 でなかった間の Status
    active = {i: [] for i in SIDES}
    received = {i: 0 for i in SIDES}

    def stamp():
        return f'{time.time() - t0:.3f}', time.strftime('%H:%M:%S')

    def on_status(i, m):
        received[i] += 1
        w.writerow([*stamp(), i, 'status', m.mode, f'{m.current:.3f}', f'{m.velocity:.2f}', f'{m.vdc:.2f}',
                    int(m.saturated_upper), int(m.saturated_lower), int(m.vdc_low), m.temperature_raw, ''])
        if target[i] != 0.0:
            active[i].append((target[i], m))

    def on_target(i, m):
        target[i] = m.data
        w.writerow([*stamp(), i, 'target', '', '', '', '', '', '', '', '', f'{m.data:.3f}'])

    def on_gripper(i, m):
        w.writerow([*stamp(), i, 'gripper', '', '', '', '', '', '', '', '', int(m.suction_on)])

    for i in SIDES:
        node.create_subscription(Status, f'/suction{i}/status', lambda m, i=i: on_status(i, m), 50)
        node.create_subscription(Float64, f'/suction{i}/target_current', lambda m, i=i: on_target(i, m), 50)
        node.create_subscription(GripperState, f'/gripper{i}/state', lambda m, i=i: on_gripper(i, m), 50)

    print(f'記録中: {args.output} (Ctrl+C で終わる)', flush=True)
    warned = False
    last_flush = t0
    try:
        while rclpy.ok() and (args.duration <= 0.0 or time.time() - t0 < args.duration):
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - last_flush > 1.0:
                f.flush()
                last_flush = now
            if not warned and now - t0 > 10.0 and not any(received.values()):
                # bringup を起動したあとでネットワークが変わると、DDS の探索が通らなくなる
                print('10 秒たっても Status が届かない。bringup は動いている? '
                      '(ネットワークを変えたなら、両方を ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST で起動し直す)', flush=True)
                warned = True
    except KeyboardInterrupt:
        pass
    finally:
        f.close()

    print(f'\n記録: {args.output} ({time.time() - t0:.0f} s)')
    for i in SIDES:
        rows = active[i]
        if not rows:
            print(f'{NAMES[i]}: Status {received[i]} 件。目標電流が 0 でない間の記録なし')
            continue
        for tgt in sorted({t for t, _ in rows}):
            cur = [m.current for t, m in rows if t == tgt]
            sat = sum(1 for t, m in rows if t == tgt and (m.saturated_upper or m.saturated_lower))
            print(f'{NAMES[i]}: 目標 {tgt:+.2f} A -> 電流 平均 {statistics.mean(cur):+.2f} A '
                  f'(標準偏差 {statistics.pstdev(cur):.2f}、{min(cur):+.2f}..{max(cur):+.2f})、'
                  f'{len(cur) / 50:.0f} s ぶん、飽和 {sat}/{len(cur)}')


if __name__ == '__main__':
    main()
