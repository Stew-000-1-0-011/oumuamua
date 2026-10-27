#!/usr/bin/env python3
"""経路の定義 (YAML) から、path_follower が読む経路ファイル (CSV: x, y, yaw) を作る。

    make_path.py route.yaml path.csv [--plot preview.png] [--reverse]

--reverse を付けると、同じ経路を終点から始点へたどる経路ファイルを作る (各点での機体の向きは同じ。
行きで前を向いて通った所を、帰りは後ろ向きに通る)。

経路の定義:

    spacing: 0.01            # 点の間隔 [m]
    corners:                 # 折れ線の頂点 [x, y] と、その角を丸める半径 [m] (両端には不要)
      - [0.0, 0.0]
      - [1.0, 0.0, 0.3]
      - [1.0, 1.0]
    yaw:                     # 始点からの道のり [m] と、そこでの機体の向き [rad]。間は滑らかにつなぐ
      - [0.0, 0.0]           # (全方向移動なので、向きは進む向きと無関係に決められる)
      - [1.5, 1.5708]
    follow_heading:          # 任意。道のり from [m] から先は、機体の向きを進む向きに合わせる
      from: 1.0              # (それまでは yaw のとおり。blend [m] かけて滑らかに移る)
      blend: 0.2
      to: 2.0                # 任意。ここで yaw のとおりに戻す (無ければ終点まで進む向き)
    robot_radius: 0.28       # 任意。機体の外形を円とみなした半径。障害物との間隔の確認と図に使う
    footprint:               # 任意。機体の外形の多角形 (機体座標系、x 前・y 左)。あれば robot_radius の代わりに使う
      - [0.3, 0.1]
      - [-0.2, 0.3]
      - [-0.2, -0.3]
      - [0.3, -0.1]
    lidar: [0.28, 0.0]       # 任意。LiDAR の取付位置 (機体座標系)。図に軌跡を描く
    min_clearance: 0.03      # 任意。外形と障害物の間に最低これだけ空ける [m] (既定 0)
    field: ../field.yaml                     # 任意。フィールドのパラメータファイル (経路の定義からの相対パス)。
                                             # その field.bounds を bounds に、field.walls と field.shelf を obstacles に使う
    bounds: [xmin, ymin, xmax, ymax]         # 任意。走れる範囲 (外周壁の内側の面)。field より優先
    obstacles:                               # 任意。軸に平行な箱 [xmin, ymin, xmax, ymax]。field のものに足す
      - [0.8, -0.1, 0.9, 0.5]

障害物を書いておくと、機体の外形 (向きも考える) から障害物までの最小の間隔を出し、
min_clearance より小さければ失敗する (ファイルは作らない)。
"""
import argparse
import math
import os
import sys

import yaml


def load_field(path):
    """フィールドのパラメータファイル (/**: ros__parameters: field: ...) から外周と障害物 (壁と棚) を読む。"""
    with open(path) as f:
        doc = yaml.safe_load(f)
    field = next(iter(doc.values()))['ros__parameters']['field']
    walls = [float(v) for v in field['walls']]
    if len(walls) % 4:
        raise ValueError(f'{path}: field.walls must be [xmin, ymin, xmax, ymax, ...]')
    boxes = [walls[i:i + 4] for i in range(0, len(walls), 4)]
    boxes.append([float(v) for v in field['shelf']])
    return {'bounds': [float(v) for v in field['bounds']], 'obstacles': boxes}


def rounded_polyline(corners, spacing):
    """角を接円弧で丸めた折れ線を spacing ごとの点列 [(x, y)] にする。"""
    points = [(c[0], c[1]) for c in corners]
    radii = [float(c[2]) if len(c) > 2 else 0.0 for c in corners]
    pieces = []          # ('line', a, b) / ('arc', centre, r, a0, sweep)
    cur = points[0]
    for i in range(1, len(points) - 1):
        a, b, c = cur, points[i], points[i + 1]
        u = (b[0] - a[0], b[1] - a[1]); lu = math.hypot(*u)
        v = (c[0] - b[0], c[1] - b[1]); lv = math.hypot(*v)
        if lu < 1e-9 or lv < 1e-9:
            raise ValueError(f'corner {i}: consecutive corners coincide')
        u = (u[0] / lu, u[1] / lu); v = (v[0] / lv, v[1] / lv)
        turn = math.atan2(u[0] * v[1] - u[1] * v[0], u[0] * v[0] + u[1] * v[1])
        r = radii[i]
        if r <= 0.0 or abs(turn) < 1e-9:
            pieces.append(('line', a, b)); cur = b
            continue
        t = r * math.tan(abs(turn) / 2)
        if t > lu + 1e-9 or t > math.hypot(c[0] - b[0], c[1] - b[1]) + 1e-9:
            raise ValueError(f'corner {i}: radius {r} does not fit between its neighbours')
        p1 = (b[0] - u[0] * t, b[1] - u[1] * t)
        p2 = (b[0] + v[0] * t, b[1] + v[1] * t)
        sgn = 1.0 if turn > 0 else -1.0
        centre = (p1[0] - sgn * u[1] * r, p1[1] + sgn * u[0] * r)
        pieces.append(('line', a, p1))
        pieces.append(('arc', centre, r, math.atan2(p1[1] - centre[1], p1[0] - centre[0]), turn))
        cur = p2
    pieces.append(('line', cur, points[-1]))

    lengths = [math.hypot(p[2][0] - p[1][0], p[2][1] - p[1][1]) if p[0] == 'line' else abs(p[4]) * p[2] for p in pieces]
    total = sum(lengths)
    n = max(1, round(total / spacing))
    out = []
    for k in range(n + 1):
        s = total * k / n
        for piece, length in zip(pieces, lengths):
            if s <= length + 1e-12 or piece is pieces[-1]:
                f = 0.0 if length < 1e-12 else min(1.0, s / length)
                if piece[0] == 'line':
                    out.append((piece[1][0] + f * (piece[2][0] - piece[1][0]), piece[1][1] + f * (piece[2][1] - piece[1][1])))
                else:
                    ang = piece[3] + f * piece[4]
                    out.append((piece[1][0] + piece[2] * math.cos(ang), piece[1][1] + piece[2] * math.sin(ang)))
                break
            s -= length
    return out, total


def yaw_at(s, keys):
    """道のり s での向き。キーの間は smoothstep でつなぎ、外側は端の値を保つ。"""
    if s <= keys[0][0]:
        return keys[0][1]
    for (s0, y0), (s1, y1) in zip(keys, keys[1:]):
        if s <= s1:
            u = (s - s0) / (s1 - s0)
            return y0 + (y1 - y0) * (u * u * (3 - 2 * u))
    return keys[-1][1]


def clearance(x, y, bounds, obstacles):
    best = float('inf')
    for x0, y0, x1, y1 in obstacles:
        best = min(best, math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1)))
    if bounds:
        best = min(best, x - bounds[0], bounds[2] - x, y - bounds[1], bounds[3] - y)
    return best


def outline(route, spacing=0.02):
    """機体の外形を、機体座標系の点列にする (多角形があればそれ、無ければ半径の円)。"""
    if route.get('footprint'):
        v = [(float(x), float(y)) for x, y in route['footprint']]
        pts = []
        for a, b in zip(v, v[1:] + v[:1]):
            n = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / spacing))
            pts += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n)]
        return pts
    r = float(route.get('robot_radius', 0.0))
    if r <= 0.0:
        return [(0.0, 0.0)]
    n = max(8, math.ceil(2 * math.pi * r / spacing))
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def to_field(pose, p):
    x, y, yaw = pose
    return (x + math.cos(yaw) * p[0] - math.sin(yaw) * p[1], y + math.sin(yaw) * p[0] + math.cos(yaw) * p[1])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('route', help='route definition (YAML)')
    parser.add_argument('output', help='path file to write (CSV)')
    parser.add_argument('--plot', help='also draw the path to this image file')
    parser.add_argument('--reverse', action='store_true', help='write the path from its last point back to its first, with the same heading at each point')
    args = parser.parse_args()

    with open(args.route) as f:
        route = yaml.safe_load(f)
    spacing = float(route.get('spacing', 0.01))
    xy, length = rounded_polyline(route['corners'], spacing)
    keys = sorted((float(s), float(y)) for s, y in route.get('yaw', [[0.0, 0.0]]))
    stations = [length * i / (len(xy) - 1) for i in range(len(xy))]
    yaws = [yaw_at(s, keys) for s in stations]
    follow = route.get('follow_heading')
    if follow:
        # 進む向き (接線)。±pi で折り返さないよう、yaw のキーの値に近い側へつなげていく
        tangent = []
        for i in range(len(xy)):
            a, b = xy[max(i - 1, 0)], xy[min(i + 1, len(xy) - 1)]
            raw = math.atan2(b[1] - a[1], b[0] - a[0])
            ref = tangent[-1] if tangent else yaws[0]
            tangent.append(raw + 2 * math.pi * round((ref - raw) / (2 * math.pi)))
        s0 = float(follow.get('from', 0.0))
        s1 = float(follow['to']) if 'to' in follow else float('inf')
        blend = max(float(follow.get('blend', 0.2)), 1e-6)
        for i, s in enumerate(stations):
            u = min(max((s - s0) / blend, 0.0), 1.0, max((s1 - s) / blend, 0.0))
            w = u * u * (3 - 2 * u)
            yaws[i] = (1 - w) * yaws[i] + w * tangent[i]
    path = [(x, y, yaw) for (x, y), yaw in zip(xy, yaws)]
    if args.reverse:
        path.reverse()

    bounds = route.get('bounds')
    obstacles = list(route.get('obstacles', []))
    if route.get('field'):
        field = load_field(os.path.join(os.path.dirname(os.path.abspath(args.route)), route['field']))
        bounds = bounds or field['bounds']
        obstacles = field['obstacles'] + obstacles
    radius = float(route.get('robot_radius', 0.0))
    margin = float(route.get('min_clearance', 0.0))
    body = outline(route)
    lidar = tuple(float(v) for v in route['lidar']) if route.get('lidar') else None
    print(f'{len(path)} points, {length:.2f} m, from ({path[0][0]:.3f}, {path[0][1]:.3f}) to ({path[-1][0]:.3f}, {path[-1][1]:.3f})')
    worst = None
    if bounds or obstacles:
        # 外形の各点から障害物までの間隔。機体の向きも入っている
        worst = (float('inf'), None, None)
        for pose in path:
            for p in body:
                q = to_field(pose, p)
                c = clearance(q[0], q[1], bounds, obstacles)
                if c < worst[0]:
                    worst = (c, pose, q)
        shape = 'footprint polygon' if route.get('footprint') else f'circle of radius {radius:.2f} m'
        print(f'min clearance of the robot outline ({shape}): {worst[0]:.3f} m, '
              f'with the robot at ({worst[1][0]:.3f}, {worst[1][1]:.3f}, yaw {worst[1][2]:.2f}), outline point ({worst[2][0]:.3f}, {worst[2][1]:.3f})')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Rectangle
        fig, ax = plt.subplots(figsize=(11, 6.6), dpi=130)
        if bounds:
            ax.add_patch(Rectangle((bounds[0], bounds[1]), bounds[2] - bounds[0], bounds[3] - bounds[1], fill=False, lw=2, ec='#333'))
        for x0, y0, x1, y1 in obstacles:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc='#444', ec='none'))
        xs, ys = [p[0] for p in path], [p[1] for p in path]
        step = max(1, round(0.25 / spacing))
        for pose in path[::step] + [path[-1]]:
            if len(body) > 1:
                ax.add_patch(Polygon([to_field(pose, p) for p in body], closed=True, fc='#2a78d6', ec='#2a78d6', alpha=0.08, lw=0.6))
            x, y, yaw = pose
            ax.annotate('', xy=(x + 0.16 * math.cos(yaw), y + 0.16 * math.sin(yaw)), xytext=(x, y),
                        arrowprops={'arrowstyle': '->', 'color': '#888', 'lw': 1.0})
        ax.plot(xs, ys, color='#2a78d6', lw=2.4, label=f'robot centre ({length:.2f} m; grey arrows: heading)')
        if lidar:
            lx, ly = zip(*(to_field(pose, lidar) for pose in path))
            ax.plot(lx, ly, color='#e07b00', lw=1.8, ls='--', label=f'LiDAR ({lidar[0]:+.2f}, {lidar[1]:+.2f} m from the centre)')
        ax.plot(xs[0], ys[0], 'o', color='#1a9c4b', ms=9, label='start')
        ax.plot(xs[-1], ys[-1], 's', color='#d6452a', ms=9, label='goal')
        if worst:
            ax.plot(worst[2][0], worst[2][1], 'x', color='k', ms=9, mew=2, label=f'closest the outline gets: {worst[0]:.2f} m')
        ax.set_aspect('equal'); ax.autoscale_view(); ax.margins(0.03)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(True, alpha=0.25)
        ax.legend(loc='lower left', fontsize=9, framealpha=0.95)
        fig.tight_layout(); fig.savefig(args.plot)
        print(f'wrote {args.plot}')

    if worst and worst[0] < margin:
        print(f'error: the robot outline comes within {worst[0]:.3f} m of an obstacle (min_clearance {margin:.2f} m); not writing {args.output}', file=sys.stderr)
        return 1

    with open(args.output, 'w') as f:
        f.write(f'# generated by make_path.py from {args.route}{" (--reverse)" if args.reverse else ""}. do not edit by hand; edit the route and regenerate\n')
        f.write(f'# {len(path)} points, {length:.3f} m\n')
        f.write('x,y,yaw\n')
        for x, y, yaw in path:
            f.write(f'{x:.5f},{y:.5f},{yaw:.5f}\n')
    print(f'wrote {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
