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
      offset: 3.14159265     # 任意。進む向きにこれ [rad] を足した向きにする (pi なら後ろ向きに進む)
    robot_radius: 0.28       # 任意。機体の外形を円とみなした半径。障害物との間隔の確認と図に使う
    footprint:               # 任意。機体の外形の多角形 (機体座標系、x 前・y 左)。あれば robot_radius の代わりに使う
      - [0.3, 0.1]
      - [-0.2, 0.3]
      - [-0.2, -0.3]
      - [0.3, -0.1]
    lidar: [0.28, 0.0]       # 任意。LiDAR の取付位置 (機体座標系)。図に軌跡を描く
    footprint_file: ../footprints/raised.yaml  # 任意。埋込 footprint の代わりに外部YAMLを読む
    footprint_yaw: 1.5707963267948966          # 原形状からbase_linkへ回す角度 [rad]
    endpoint_footprint_file: ../footprints/lowered.yaml  # 同じ変換で、両端の下形状も検証する
    min_clearance: 0.03      # 任意。外形と障害物の間に最低これだけ空ける [m] (既定 0)
    field: ../field.yaml                     # 任意。フィールドのパラメータファイル (経路の定義からの相対パス)。
                                             # その field.bounds を bounds に、field.walls と field.shelf を obstacles に使う
    field_wall_padding: 0.0025                # 任意。fieldの内壁だけを各面から拡大する [m]
    field_bounds_padding: 0.005               # 任意。fieldの外周内面を内側へ移す [m]
    bounds: [xmin, ymin, xmax, ymax]         # 任意。走れる範囲 (外周壁の内側の面)。field より優先
    obstacles:                               # 任意。軸に平行な箱 [xmin, ymin, xmax, ymax]。field のものに足す
      - [0.8, -0.1, 0.9, 0.5]

障害物を書いておくと、多角形の包含・辺交差と点間の位置/yaw補間を含めて検証する。
表示する swept clearance は全区間の間隔の安全側の下限。min_clearance 未満なら失敗し、CSVは上書きしない。
"""
import argparse
import math
import os
import sys

import yaml


def load_field(path, wall_padding=0.0, bounds_padding=0.0):
    """フィールドのパラメータファイル (/**: ros__parameters: field: ...) から外周と障害物 (壁と棚) を読む。"""
    with open(path) as f:
        doc = yaml.safe_load(f)
    field = next(iter(doc.values()))['ros__parameters']['field']
    walls = [float(v) for v in field['walls']]
    if len(walls) % 4:
        raise ValueError(f'{path}: field.walls must be [xmin, ymin, xmax, ymax, ...]')
    boxes = [[walls[i] - wall_padding, walls[i + 1] - wall_padding,
              walls[i + 2] + wall_padding, walls[i + 3] + wall_padding]
             for i in range(0, len(walls), 4)]
    boxes.append([float(v) for v in field['shelf']])
    x0, y0, x1, y1 = map(float, field['bounds'])
    return {'bounds': [x0 + bounds_padding, y0 + bounds_padding,
                       x1 - bounds_padding, y1 - bounds_padding], 'obstacles': boxes}


def polygon_points(value, yaw=0.0):
    """Read either polygon YAML syntax, preserving concavities and source axes."""
    if isinstance(value, str):
        value = yaml.safe_load(value)
    try:
        points = [(float(x), float(y)) for x, y in value]
    except (TypeError, ValueError) as e:
        raise ValueError('footprint must contain [x, y] pairs') from e
    if len(points) < 3 or not math.isfinite(yaw) or not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('footprint needs at least three finite vertices and a finite rotation')
    c, s = math.cos(yaw), math.sin(yaw)
    return [(c * x - s * y, s * x + c * y) for x, y in points]


def load_footprint(path, yaw=0.0):
    with open(path) as f:
        doc = yaml.safe_load(f)
    return polygon_points(doc['footprint'] if isinstance(doc, dict) else doc, yaw)


def load_route(path):
    with open(path) as f:
        route = yaml.safe_load(f)
    directory = os.path.dirname(os.path.abspath(path))
    yaw = float(route.get('footprint_yaw', 0.0))
    if route.get('footprint_file'):
        if route.get('footprint'):
            raise ValueError('choose footprint_file or footprint, not both')
        route['footprint'] = load_footprint(os.path.join(directory, route['footprint_file']), yaw)
    elif route.get('footprint'):
        route['footprint'] = polygon_points(route['footprint'], yaw)
    if route.get('endpoint_footprint_file'):
        route['_endpoint_footprint'] = load_footprint(os.path.join(directory, route['endpoint_footprint_file']), yaw)
    for key in ('field_wall_padding', 'field_bounds_padding', 'robot_radius', 'min_clearance'):
        value = float(route.get(key, 0.0))
        if not math.isfinite(value) or value < 0:
            raise ValueError(f'{key} must be finite and nonnegative')
    return route


def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (a[1] > y) != (b[1] > y):
            if x < a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
                inside = not inside
    return inside


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2)) if length2 else 0.0
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def segments_intersect(a, b, c, d):
    def cross(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    if max(a[0], b[0]) < min(c[0], d[0]) or max(c[0], d[0]) < min(a[0], b[0]):
        return False
    if max(a[1], b[1]) < min(c[1], d[1]) or max(c[1], d[1]) < min(a[1], b[1]):
        return False
    return cross(a, b, c) * cross(a, b, d) <= 0 and cross(c, d, a) * cross(c, d, b) <= 0


def polygon_clearance(polygon, bounds, obstacles):
    """Exact polygon/axis-aligned rectangle distance, zero on containment/contact.

    Negative values indicate an outline beyond the outer bounds. Bounding boxes
    only prune distant obstacles; they never replace the concave body polygon.
    """
    xmin = min(p[0] for p in polygon); xmax = max(p[0] for p in polygon)
    ymin = min(p[1] for p in polygon); ymax = max(p[1] for p in polygon)
    best = min(xmin - bounds[0], bounds[2] - xmax, ymin - bounds[1], bounds[3] - ymax) if bounds else float('inf')
    if best <= 0:
        return best
    edges = list(zip(polygon, polygon[1:] + polygon[:1]))
    for x0, y0, x1, y1 in obstacles:
        if math.hypot(max(x0 - xmax, xmin - x1, 0), max(y0 - ymax, ymin - y1, 0)) >= best:
            continue
        box = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in polygon) or any(point_in_polygon(p, polygon) for p in box):
            return 0.0
        for a, b in edges:
            if any(segments_intersect(a, b, c, d) for c, d in zip(box, box[1:] + box[:1])):
                return 0.0
        # Disjoint segment distance attains its minimum at an endpoint of one
        # segment. Polygon vertices -> box, and box vertices -> polygon edges.
        best = min(best, *(math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)) for x, y in polygon))
        best = min(best, *(point_segment_distance(p, a, b) for p in box for a, b in edges))
    return best


def pose_clearance(pose, route, bounds, obstacles):
    if route.get('footprint'):
        return polygon_clearance([to_field(pose, p) for p in route['footprint']], bounds, obstacles)
    return clearance(pose[0], pose[1], bounds, obstacles) - float(route.get('robot_radius', 0.0))


def validate_path(path, route, bounds, obstacles):
    """Conservative clearance of the CSV's linear position and unwrapped yaw.

    Any body point moves at most |translation| + R*|delta yaw|. Subdivide
    until this bound is <=4 mm; every intermediate pose is within <=2 mm
    of a checked pose. Subtract half the bound from the smaller endpoint
    clearance of each subinterval. Thus the result bounds the whole sweep,
    rather than only a set of samples (also for concave footprints).
    """
    if not path or not all(math.isfinite(v) for pose in path for v in pose):
        raise ValueError('path needs finite poses')
    radius = max((math.hypot(*p) for p in route['footprint']), default=0.0) if route.get('footprint') else float(route.get('robot_radius', 0.0))
    worst = (pose_clearance(path[0], route, bounds, obstacles), path[0])
    for a, b in zip(path, path[1:]):
        translation = math.hypot(b[0] - a[0], b[1] - a[1])
        motion = translation + radius * abs(b[2] - a[2])
        n = max(1, math.ceil(motion / .004))
        previous = pose_clearance(a, route, bounds, obstacles)
        for k in range(1, n + 1):
            u = k / n
            pose = tuple(v + u * (w - v) for v, w in zip(a, b))
            current = pose_clearance(pose, route, bounds, obstacles)
            bound = min(previous, current) - motion / (2 * n)
            if bound < worst[0]:
                worst = (bound, pose)
            previous = current
    return worst


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

    route = load_route(args.route)
    spacing = float(route.get('spacing', 0.01))
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('spacing must be finite and positive')
    xy, length = rounded_polyline(route['corners'], spacing)
    keys = sorted((float(s), float(y)) for s, y in route.get('yaw', [[0.0, 0.0]]))
    stations = [length * i / (len(xy) - 1) for i in range(len(xy))]
    yaws = [yaw_at(s, keys) for s in stations]
    follow = route.get('follow_heading')
    if follow:
        offset = float(follow.get('offset', 0.0))
        # 進む向き (接線) + offset。±pi で折り返さないよう、yaw のキーの値に近い側へつなげていく
        tangent = []
        for i in range(len(xy)):
            a, b = xy[max(i - 1, 0)], xy[min(i + 1, len(xy) - 1)]
            raw = math.atan2(b[1] - a[1], b[0] - a[0]) + offset
            ref = tangent[-1] if tangent else yaws[0]
            tangent.append(raw + 2 * math.pi * round((ref - raw) / (2 * math.pi)))
        s0 = float(follow.get('from', 0.0))
        s1 = float(follow['to']) if 'to' in follow else float('inf')
        blend = max(float(follow.get('blend', 0.2)), 1e-6)
        for i, s in enumerate(stations):
            u = min(max((s - s0) / blend, 0.0), 1.0, max((s1 - s) / blend, 0.0))
            w = u * u * (3 - 2 * u)
            yaws[i] = (1 - w) * yaws[i] + w * tangent[i]
    # Validate exactly the poses consumed by trajectory.cpp, after CSV rounding.
    path = [tuple(round(v, 5) for v in (x, y, yaw)) for (x, y), yaw in zip(xy, yaws)]
    if args.reverse:
        path.reverse()

    bounds = route.get('bounds')
    obstacles = list(route.get('obstacles', []))
    if route.get('field'):
        field = load_field(os.path.join(os.path.dirname(os.path.abspath(args.route)), route['field']),
                           float(route.get('field_wall_padding', 0.0)), float(route.get('field_bounds_padding', 0.0)))
        bounds = bounds or field['bounds']
        obstacles = field['obstacles'] + obstacles
    radius = float(route.get('robot_radius', 0.0))
    margin = float(route.get('min_clearance', 0.0))
    body = outline(route)
    lidar = tuple(float(v) for v in route['lidar']) if route.get('lidar') else None
    print(f'{len(path)} points, {length:.2f} m, from ({path[0][0]:.3f}, {path[0][1]:.3f}) to ({path[-1][0]:.3f}, {path[-1][1]:.3f})')
    worst = None
    endpoint_worst = None
    if bounds or obstacles:
        worst = validate_path(path, route, bounds, obstacles)
        shape = 'footprint polygon' if route.get('footprint') else f'circle of radius {radius:.2f} m'
        print(f'conservative swept clearance ({shape}): {worst[0]:.6f} m, '
              f'near robot pose ({worst[1][0]:.3f}, {worst[1][1]:.3f}, yaw {worst[1][2]:.2f})')
        if route.get('_endpoint_footprint'):
            low = {'footprint': route['_endpoint_footprint']}
            endpoint_worst = min((pose_clearance(pose, low, bounds, obstacles), pose) for pose in (path[0], path[-1]))
            print(f'lowered endpoint clearance: {endpoint_worst[0]:.6f} m')

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
        if route.get('_endpoint_footprint'):
            for i, pose in enumerate((path[0], path[-1])):
                ax.add_patch(Polygon([to_field(pose, p) for p in route['_endpoint_footprint']],
                                     closed=True, fill=False, ec='#923fad', lw=1.5,
                                     label='lowered body at endpoints' if i == 0 else None))
        ax.plot(xs, ys, color='#2a78d6', lw=2.4, label=f'robot centre ({length:.2f} m; grey arrows: heading)')
        if lidar:
            lx, ly = zip(*(to_field(pose, lidar) for pose in path))
            ax.plot(lx, ly, color='#e07b00', lw=1.8, ls='--', label=f'LiDAR ({lidar[0]:+.2f}, {lidar[1]:+.2f} m from the centre)')
        ax.plot(xs[0], ys[0], 'o', color='#1a9c4b', ms=9, label='start')
        ax.plot(xs[-1], ys[-1], 's', color='#d6452a', ms=9, label='goal')
        if worst:
            ax.plot(worst[1][0], worst[1][1], 'x', color='k', ms=9, mew=2, label=f'limiting robot pose; swept clearance >= {worst[0]:.3f} m')
        ax.set_aspect('equal'); ax.autoscale_view(); ax.margins(0.03)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(True, alpha=0.25)
        ax.legend(loc='lower left', fontsize=9, framealpha=0.95)
        fig.tight_layout(); fig.savefig(args.plot)
        print(f'wrote {args.plot}')

    if (worst and worst[0] < margin) or (endpoint_worst and endpoint_worst[0] < margin):
        print(f'error: swept body or lowered endpoint violates min_clearance {margin:.3f} m; not writing {args.output}', file=sys.stderr)
        return 1

    with open(args.output, 'w') as f:
        f.write(f'# generated by make_path.py from {os.path.basename(args.route)}{" (--reverse)" if args.reverse else ""}. do not edit by hand; edit the route and regenerate\n')
        f.write(f'# {len(path)} points, {length:.3f} m\n')
        f.write('x,y,yaw\n')
        for x, y, yaw in path:
            f.write(f'{x:.5f},{y:.5f},{yaw:.5f}\n')
    print(f'wrote {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
