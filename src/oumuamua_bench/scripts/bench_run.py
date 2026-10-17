#!/usr/bin/env python3
"""構成どうしを閉ループで比べる。

構成 (config/variants.yaml) x 状況 (config/scenarios.yaml) x 乱数の種 ごとに
`bringup.launch.py sim:=true` を立て、path_follower で本番の経路を往復させて
(start_to_shelf -> shelf_to_start)、次を測る。結果は 1 走行 1 行の JSON で <out>/runs.jsonl に足していく。

- 追従: 経路からのずれ (真値と経路の距離)、目標からの遅れ (真値と path_follower の目標の距離)、
  所要時間、終点での誤差、壁との最小の間隔 (機体の外形と地図の点)
- 自己位置: TF field -> base_link の推定と真値の差。スキャン時刻での誤差 (推定が指す時刻の真値と比べる) と、
  「今」の誤差 (真値の各時刻に、そこまでに届いた最新の推定と比べる)
- 計算負荷: ノードの実行ファイルごとの CPU 時間 (走っている間)・最大メモリ。自己位置推定 / 経路追従 / その他にまとめる

表にするのは bench_summary.py。

例:
  ros2 run oumuamua_bench bench_run.py --out /ws/bench/run1 --variants sotoba_est sotoba_pp --scenarios nominal tilt --seeds 3
"""

import argparse
import bisect
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from holonomic_tracker.msg import TrackingReference
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

try:
    from sotoba_ros.msg import BeliefArray
except ImportError:  # sotoba_ros が無くても動くように
    BeliefArray = None

LEGS = ['start_to_shelf', 'shelf_to_start']

# 実行ファイル名 -> まとめる区分。ここに無いものは other (robot_sim と launch 自身は数えない)
GROUPS = {
    'localization': [
        'sotoba_node', 'persistence_predictor', 'state_estimator_node', 'body_odometry_node',
        'amcl', 'map_server', 'gicp_localizer', 'emcl2_node',
    ],
    'navigation': [
        'tracker_node', 'path_follower_node', 'controller_server', 'planner_server', 'bt_navigator',
        'velocity_smoother', 'behavior_server', 'smoother_server', 'waypoint_follower', 'collision_monitor',
        'route_server', 'docking_server', 'opennav_docking', 'nav2_path_driver.py',
    ],
}
IGNORED = {'robot_sim', 'ros2', 'bench_run.py', 'python3', 'bash', 'sh', 'sleep', 'podman', 'conmon'}


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# --- 計算負荷 (/proc) ---

CLK_TCK = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')


def ros_processes():
    """pid -> 実行ファイル名。python のノードはスクリプト名にする"""
    result = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            argv = Path(f'/proc/{entry}/cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if not argv or not argv[0]:
            continue
        name = os.path.basename(argv[0].decode(errors='replace'))
        if name.startswith('python') and len(argv) > 1 and argv[1]:
            name = os.path.basename(argv[1].decode(errors='replace'))
        if name in IGNORED or int(entry) == os.getpid():
            continue
        # ROS のノードだけ (引数に --ros-args がある)
        if b'--ros-args' not in argv:
            continue
        result[int(entry)] = name
    return result


def cpu_seconds(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        # fields[0] が state (stat の 3 番目)。utime / stime は 14 / 15 番目
        return (int(fields[11]) + int(fields[12])) / CLK_TCK, int(fields[21]) * PAGE
    except (OSError, IndexError, ValueError):
        return None


class CpuMeter:
    def __init__(self):
        self.start = {}
        self.names = {}
        self.peak_rss = {}

    def begin(self):
        self.names = ros_processes()
        self.start = {pid: cpu_seconds(pid) for pid in self.names}
        self.t0 = time.monotonic()

    def sample(self):
        for pid in self.names:
            value = cpu_seconds(pid)
            if value:
                self.peak_rss[pid] = max(self.peak_rss.get(pid, 0), value[1])

    def end(self):
        wall = time.monotonic() - self.t0
        per_exe = {}
        for pid, name in self.names.items():
            before = self.start.get(pid)
            after = cpu_seconds(pid)
            if not before or not after:
                continue
            entry = per_exe.setdefault(name, {'cpu_s': 0.0, 'rss_mb': 0.0})
            entry['cpu_s'] += after[0] - before[0]
            entry['rss_mb'] += self.peak_rss.get(pid, after[1]) / 2**20
        groups = {}
        for name, entry in per_exe.items():
            group = next((g for g, names in GROUPS.items() if name in names), 'other')
            g = groups.setdefault(group, {'cpu_s': 0.0, 'rss_mb': 0.0})
            g['cpu_s'] += entry['cpu_s']
            g['rss_mb'] += entry['rss_mb']
        for g in groups.values():
            g['cpu_percent'] = 100.0 * g['cpu_s'] / wall if wall > 0 else float('nan')
        return {'wall_s': wall, 'per_exe': per_exe, 'groups': groups}


# --- 記録 ---

class Recorder(Node):
    def __init__(self, field_frame='field', base_frame='base_link'):
        super().__init__('bench_recorder')
        self.field_frame = field_frame
        self.base_frame = base_frame
        self.truth = []   # (t, x, y, yaw)
        self.est = []     # (推定が指す時刻, 受け取った時刻, x, y, yaw)
        self.ref = []     # (受け取った時刻, x, y, yaw, vx, vy, omega)
        self.cmd = []     # (受け取った時刻, vx, vy, omega)
        self.scan_count = 0
        # 自己位置推定が推定を出した回数 (更新の回数)。どれか 1 つだけが出しているはず
        self.update_count = 0
        self.recording = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, '/robot_sim/truth_pose', self.on_truth, 100)
        self.create_subscription(TrackingReference, '/tracker_node/reference', self.on_ref, 100)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 100)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        for topic in ('/amcl_pose', '/gicp_localizer/pose', '/mcl_pose'):
            self.create_subscription(PoseWithCovarianceStamped, topic, self.on_update, 100)
        if BeliefArray is not None:
            self.create_subscription(BeliefArray, '/sotoba_node/posterior_beliefs', self.on_update, 100)
        self.last_est_stamp = None
        self.create_timer(0.01, self.poll_tf)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, m):
        if self.recording:
            p = m.pose.position
            self.truth.append((stamp_sec(m.header.stamp), p.x, p.y, yaw_of(m.pose.orientation)))

    def on_ref(self, m):
        self.last_ref = (self.now_s(), m.x, m.y, m.yaw, m.vx, m.vy, m.omega)
        if self.recording:
            self.ref.append(self.last_ref)

    def on_cmd(self, m):
        if self.recording:
            t = m.twist
            self.cmd.append((self.now_s(), t.linear.x, t.linear.y, t.angular.z))

    def on_scan(self, _):
        if self.recording:
            self.scan_count += 1

    def on_update(self, _):
        if self.recording:
            self.update_count += 1

    def poll_tf(self):
        # 構成によって TF の鎖が違う (field -> base_link 直か、field -> odom -> base_link か) ので、
        # 鎖をたどった最新の推定を取る
        try:
            t = self.tf_buffer.lookup_transform(self.field_frame, self.base_frame, Time())
        except Exception:  # noqa: BLE001  (まだ来ていない・鎖が切れている)
            return
        stamp = stamp_sec(t.header.stamp)
        if stamp == self.last_est_stamp:
            return
        self.last_est_stamp = stamp
        if self.recording:
            a = t.transform
            self.est.append((stamp, self.now_s(), a.translation.x, a.translation.y, yaw_of(a.rotation)))

    def has_estimate(self):
        return self.last_est_stamp is not None


def spin_for(node, seconds, meter=None):
    end = time.monotonic() + seconds
    next_sample = 0.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
        if meter and time.monotonic() >= next_sample:
            meter.sample()
            next_sample = time.monotonic() + 1.0


def call_trigger(node, name, timeout):
    client = node.create_client(Trigger, name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return False, 'service not available'
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            return False, 'no response'
        return future.result().success, future.result().message
    finally:
        node.destroy_client(client)


# --- 幾何 ---

def load_path(path_dir, name):
    rows = []
    for line in Path(path_dir, f'{name}.csv').read_text().splitlines():
        if not line or line.startswith('#') or line[0].isalpha():
            continue
        rows.append([float(v) for v in line.split(',')])
    return np.array(rows)


def distance_to_polyline(points, poly):
    """各点から折れ線までの距離"""
    a = poly[:-1, :2]
    b = poly[1:, :2]
    ab = b - a
    len2 = np.maximum((ab * ab).sum(axis=1), 1e-12)
    result = np.empty(len(points))
    for i, p in enumerate(points):
        t = np.clip(((p - a) * ab).sum(axis=1) / len2, 0.0, 1.0)
        d = a + t[:, None] * ab - p
        result[i] = np.sqrt((d * d).sum(axis=1).min())
    return result


def load_map_points(pcd):
    lines = Path(pcd).read_text().splitlines()
    start = next(i for i, line in enumerate(lines) if line.startswith('DATA')) + 1
    return np.array([[float(v) for v in line.split()[:2]] for line in lines[start:] if line.strip()])


def clearance(pose, footprint, map_points):
    """機体の外形 (多角形) から地図の点までの最小の距離。点が外形の中にあれば負"""
    x, y, yaw = pose
    near = map_points[np.hypot(map_points[:, 0] - x, map_points[:, 1] - y) < 1.0]
    if len(near) == 0:
        return float('inf')
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.column_stack(((near[:, 0] - x) * c + (near[:, 1] - y) * s,
                             -(near[:, 0] - x) * s + (near[:, 1] - y) * c))
    a = footprint
    b = np.roll(footprint, -1, axis=0)
    best = float('inf')
    for p in local:
        ab = b - a
        t = np.clip(((p - a) * ab).sum(axis=1) / np.maximum((ab * ab).sum(axis=1), 1e-12), 0.0, 1.0)
        d = np.sqrt(((a + t[:, None] * ab - p) ** 2).sum(axis=1)).min()
        # 中にあるか (偶奇)
        crossing = ((a[:, 1] > p[1]) != (b[:, 1] > p[1])) & (
            p[0] < a[:, 0] + (p[1] - a[:, 1]) * (b[:, 0] - a[:, 0]) / np.where(b[:, 1] != a[:, 1], b[:, 1] - a[:, 1], 1e-12))
        if crossing.sum() % 2 == 1:
            d = -d
        best = min(best, d)
    return best


def stats(values, prefix):
    v = np.asarray(values, dtype=float)
    if v.size == 0:
        return {f'{prefix}_rms': float('nan'), f'{prefix}_p95': float('nan'), f'{prefix}_max': float('nan')}
    return {
        f'{prefix}_rms': float(np.sqrt(np.mean(v * v))),
        f'{prefix}_p95': float(np.percentile(v, 95)),
        f'{prefix}_max': float(v.max()),
    }


def compute_metrics(rec, legs, paths, map_points, footprint):
    truth = np.array(rec.truth) if rec.truth else np.zeros((0, 4))
    est = np.array(rec.est) if rec.est else np.zeros((0, 5))
    ref = np.array(rec.ref) if rec.ref else np.zeros((0, 7))
    cmd = np.array(rec.cmd) if rec.cmd else np.zeros((0, 4))
    m = {}

    # 追従 (区間ごとに経路が違う)
    path_dev = []
    lag = []
    for leg in legs:
        name, t0, t1 = leg['name'], leg['t_start'], leg['t_end']
        sel = (truth[:, 0] >= t0) & (truth[:, 0] <= t1)
        tr = truth[sel]
        if len(tr):
            path_dev.extend(distance_to_polyline(tr[:, 1:3], paths[name]).tolist())
        if len(tr) and len(ref):
            for t, x, y, _ in tr:
                i = bisect.bisect_right(ref[:, 0], t) - 1
                if i >= 0:
                    lag.append(math.hypot(ref[i, 1] - x, ref[i, 2] - y))
        m[f'{name}_time_s'] = leg.get('arrival_s', float('nan'))
        m[f'{name}_goal_err_m'] = leg.get('goal_err_m', float('nan'))
        m[f'{name}_goal_err_yaw'] = leg.get('goal_err_yaw', float('nan'))
        m[f'{name}_ok'] = leg['ok']
    m.update(stats(path_dev, 'path_dev'))
    m.update(stats(lag, 'ref_lag'))

    # 壁との間隔 (20 Hz に間引く)
    if len(truth):
        step = max(1, len(truth) // max(1, int((truth[-1, 0] - truth[0, 0]) * 20)))
        m['clearance_min'] = float(min(clearance(p[1:4], footprint, map_points) for p in truth[::step]))
    else:
        m['clearance_min'] = float('nan')

    # 自己位置
    at_stamp = []
    at_stamp_yaw = []
    if len(truth) > 1:
        for t, _, x, y, w in est:
            i = bisect.bisect_left(truth[:, 0], t)
            if 0 < i < len(truth):
                a, b = truth[i - 1], truth[i]
                r = (t - a[0]) / max(b[0] - a[0], 1e-9)
                gx = a[1] + r * (b[1] - a[1])
                gy = a[2] + r * (b[2] - a[2])
                gw = a[3] + r * wrap(b[3] - a[3])
                at_stamp.append(math.hypot(x - gx, y - gy))
                at_stamp_yaw.append(abs(wrap(w - gw)))
    now = []
    now_yaw = []
    if len(est):
        for t, x, y, w in truth:
            i = bisect.bisect_right(est[:, 1], t) - 1
            if i >= 0:
                now.append(math.hypot(est[i, 2] - x, est[i, 3] - y))
                now_yaw.append(abs(wrap(est[i, 4] - w)))
    m.update(stats(at_stamp, 'est_err'))
    m.update(stats(at_stamp_yaw, 'est_err_yaw'))
    m.update(stats(now, 'now_err'))
    m.update(stats(now_yaw, 'now_err_yaw'))
    m['est_rate_hz'] = float(len(est) / max(est[-1, 1] - est[0, 1], 1e-9)) if len(est) > 1 else 0.0
    m['lost'] = bool(at_stamp and max(at_stamp) > 0.15)

    # 指令の滑らかさ: 並進速度の指令の変化率の rms [m/s^2]
    if len(cmd) > 2:
        dt = np.diff(cmd[:, 0])
        ok = dt > 1e-4
        acc = np.hypot(np.diff(cmd[:, 1])[ok], np.diff(cmd[:, 2])[ok]) / dt[ok]
        m['cmd_accel_rms'] = float(np.sqrt(np.mean(acc * acc))) if acc.size else float('nan')
    else:
        m['cmd_accel_rms'] = float('nan')
    m['scan_count'] = rec.scan_count
    m['update_count'] = rec.update_count
    return m


# --- 1 走行 ---

def nested(flat):
    """'tilt.amplitude': 0.1 -> {'tilt': {'amplitude': 0.1}}"""
    out = {}
    for key, value in flat.items():
        d = out
        parts = key.split('.')
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = value
    return out


def kill_leftovers():
    names = {n for ns in GROUPS.values() for n in ns} | {'robot_sim', 'static_transform_publisher', 'lifecycle_manager'}
    for pid, name in ros_processes().items():
        if name in names:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass


def run_once(args, variant_name, variant, scenario_name, scenario, seed, paths, map_points, footprint):
    out = Path(args.out)
    log_dir = out / 'logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    tag = f'{variant_name}__{scenario_name}__{seed}'

    sim_params = {'seed': seed}
    sim_params.update(scenario.get('robot_sim') or {})
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix='bench_sim_', delete=False) as f:
        yaml.safe_dump({'robot_sim': {'ros__parameters': nested(sim_params)}}, f)
        sim_file = f.name

    cmd = ['ros2', 'launch', 'oumuamua_bringup', 'bringup.launch.py', 'sim:=true', f'sim_params:={sim_file}']
    cmd += list(variant.get('launch_args') or [])
    # 構成ごとのパラメータの上書き (variants の params: {ノード名: {パラメータ: 値}})
    overrides_file = None
    if variant.get('params'):
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix='bench_overrides_', delete=False) as f:
            yaml.safe_dump({node: {'ros__parameters': nested(p)} for node, p in variant['params'].items()}, f)
            overrides_file = f.name
        cmd.append(f'overrides:={overrides_file}')
    log = open(log_dir / f'{tag}.log', 'w')
    launch = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    rec = Recorder()
    meter = CpuMeter()
    legs = []
    error = None
    cpu = {'wall_s': 0.0, 'per_exe': {}, 'groups': {}}
    try:
        # 推定が出て、path_follower が走り出せるまで待つ
        deadline = time.monotonic() + args.startup_timeout
        while not rec.has_estimate() and time.monotonic() < deadline:
            rclpy.spin_once(rec, timeout_sec=0.05)
        spin_for(rec, args.settle)
        rec.recording = True
        meter.begin()
        for name in LEGS:
            path = paths[name]
            goal = path[-1]
            ok, msg = False, ''
            deadline = time.monotonic() + (args.startup_timeout if not legs else 5.0)
            while time.monotonic() < deadline:
                ok, msg = call_trigger(rec, f'/path_follower/start/{name}', 2.0)
                if ok:
                    break
                spin_for(rec, 0.5, meter)
            t_start = rec.now_s()
            leg = {'name': name, 't_start': t_start, 'ok': False}
            legs.append(leg)
            if not ok:
                leg['t_end'] = rec.now_s()
                error = f'{name}: start refused: {msg}'
                break
            # 目標が終点に着き、そこから settle 秒たつまで
            leg_deadline = time.monotonic() + args.leg_timeout
            reached_ref = None
            while time.monotonic() < leg_deadline:
                spin_for(rec, 0.05, meter)
                r = getattr(rec, 'last_ref', None)
                if reached_ref is None and r and r[0] > t_start and math.hypot(r[1] - goal[0], r[2] - goal[1]) < 1e-3 \
                        and abs(r[4]) + abs(r[5]) + abs(r[6]) < 1e-6:
                    reached_ref = time.monotonic()
                if reached_ref is not None and time.monotonic() - reached_ref >= args.goal_settle:
                    break
            leg['t_end'] = rec.now_s()
            tr = [p for p in rec.truth if p[0] >= t_start]
            if tr:
                last = tr[-1]
                leg['goal_err_m'] = math.hypot(last[1] - goal[0], last[2] - goal[1])
                leg['goal_err_yaw'] = abs(wrap(last[3] - goal[2]))
                # 着いた時刻: 終点から 5 cm 以内に入って、以後出なかった最初の時刻
                arrival = None
                for p in tr:
                    inside = math.hypot(p[1] - goal[0], p[2] - goal[1]) < 0.05
                    if inside and arrival is None:
                        arrival = p[0]
                    elif not inside:
                        arrival = None
                leg['arrival_s'] = (arrival - t_start) if arrival is not None else float('nan')
            leg['ok'] = reached_ref is not None and leg.get('goal_err_m', 1.0) < 0.05
            if reached_ref is None:
                error = f'{name}: timed out'
                break
        cpu = meter.end()
    finally:
        rec.recording = False
        try:
            os.killpg(launch.pid, signal.SIGINT)
            launch.wait(timeout=15)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(launch.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        log.close()
        kill_leftovers()
        os.unlink(sim_file)
        if overrides_file:
            os.unlink(overrides_file)

    metrics = compute_metrics(rec, legs, paths, map_points, footprint)
    if args.save_traces:
        (out / 'traces').mkdir(exist_ok=True)
        with open(out / 'traces' / f'{tag}.json', 'w') as f:
            json.dump({'truth': rec.truth, 'est': rec.est, 'ref': rec.ref, 'cmd': rec.cmd, 'legs': legs}, f)
    rec.destroy_node()
    result = {
        'variant': variant_name,
        'scenario': scenario_name,
        'seed': seed,
        'ok': error is None and all(leg['ok'] for leg in legs) and len(legs) == len(LEGS),
        'error': error,
        'metrics': metrics,
        'cpu': cpu,
    }
    with open(out / 'runs.jsonl', 'a') as f:
        f.write(json.dumps(result) + '\n')
    return result


def main():
    share = Path(get_package_share_directory('oumuamua_bench'))
    bringup = Path(get_package_share_directory('oumuamua_bringup'))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', required=True, help='結果を置くディレクトリ (runs.jsonl に追記する)')
    parser.add_argument('--variants', nargs='+', help='構成 (既定: variants.yaml の全部)')
    parser.add_argument('--scenarios', nargs='+', help='状況 (既定: scenarios.yaml の全部)')
    parser.add_argument('--seeds', type=int, default=3, help='乱数の種の数 (0 から)')
    parser.add_argument('--seed-offset', type=int, default=0)
    parser.add_argument('--variants-file', default=str(share / 'config' / 'variants.yaml'))
    parser.add_argument('--scenarios-file', default=str(share / 'config' / 'scenarios.yaml'))
    parser.add_argument('--path-dir', default=str(bringup / 'config' / 'paths'))
    parser.add_argument('--map', default=str(bringup / 'config' / 'maps' / 'field.pcd'))
    parser.add_argument('--startup-timeout', type=float, default=30.0)
    parser.add_argument('--settle', type=float, default=2.0, help='推定が出てから走り出すまでの待ち [s]')
    parser.add_argument('--leg-timeout', type=float, default=60.0)
    parser.add_argument('--save-traces', action='store_true',
                        help='走行ごとの真値・推定・目標・指令を <out>/traces/ に JSON で残す')
    parser.add_argument('--goal-settle', type=float, default=1.0, help='目標が終点に着いてから測るまでの待ち [s]')
    args = parser.parse_args()

    variants = yaml.safe_load(Path(args.variants_file).read_text())
    scenarios = yaml.safe_load(Path(args.scenarios_file).read_text())
    variant_names = args.variants or list(variants)
    scenario_names = args.scenarios or list(scenarios)
    paths = {name: load_path(args.path_dir, name) for name in LEGS}
    map_points = load_map_points(args.map)
    route = yaml.safe_load(Path(args.path_dir, 'start_to_shelf.route.yaml').read_text())
    footprint = np.array(route['footprint'], dtype=float)

    rclpy.init()
    kill_leftovers()
    total = len(variant_names) * len(scenario_names) * args.seeds
    count = 0
    for seed in range(args.seed_offset, args.seed_offset + args.seeds):
        for scenario_name in scenario_names:
            for variant_name in variant_names:
                count += 1
                r = run_once(args, variant_name, variants[variant_name], scenario_name, scenarios[scenario_name],
                             seed, paths, map_points, footprint)
                mm = r['metrics']
                loc = r['cpu']['groups'].get('localization', {}).get('cpu_percent', float('nan'))
                print(f"[{count}/{total}] {variant_name:14s} {scenario_name:9s} seed={seed} "
                      f"{'ok  ' if r['ok'] else 'FAIL'} path_dev rms {mm['path_dev_rms'] * 1000:5.1f} mm "
                      f"max {mm['path_dev_max'] * 1000:5.1f} | est rms {mm['est_err_rms'] * 1000:5.1f} mm "
                      f"| clearance {mm['clearance_min'] * 1000:6.1f} mm | loc cpu {loc:5.1f} %"
                      f"{'  ' + r['error'] if r['error'] else ''}", flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
