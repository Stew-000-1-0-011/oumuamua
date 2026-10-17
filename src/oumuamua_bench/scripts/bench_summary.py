#!/usr/bin/env python3
"""bench_run.py の runs.jsonl を、構成 x 状況ごとの表 (Markdown) にする。

各欄は乱数の種ごとの値の平均。± は標準偏差。失敗は「成功した走行 / 走行数」で出す。

例:
  ros2 run oumuamua_bench bench_summary.py /ws/bench/run1/runs.jsonl > summary.md
"""

import argparse
import json
import math
from collections import OrderedDict

# (見出し, 取り出し方, 倍率, 小数の桁)
COLUMNS = [
    ('経路からのずれ rms [mm]', lambda r: r['metrics']['path_dev_rms'], 1000, 1),
    ('経路からのずれ 最大 [mm]', lambda r: r['metrics']['path_dev_max'], 1000, 1),
    ('壁との間隔 最小 [mm]', lambda r: r['metrics']['clearance_min'], 1000, 0),
    ('往路 [s]', lambda r: r['metrics'].get('start_to_shelf_time_s'), 1, 2),
    ('復路 [s]', lambda r: r['metrics'].get('shelf_to_start_time_s'), 1, 2),
    ('終点の誤差 [mm]', lambda r: max(r['metrics'].get('start_to_shelf_goal_err_m', math.nan),
                                    r['metrics'].get('shelf_to_start_goal_err_m', math.nan)), 1000, 1),
    ('推定誤差 rms [mm]', lambda r: r['metrics']['est_err_rms'], 1000, 1),
    ('推定誤差 最大 [mm]', lambda r: r['metrics']['est_err_max'], 1000, 1),
    ('向きの推定誤差 最大 [mrad]', lambda r: r['metrics']['est_err_yaw_max'], 1000, 1),
    ('「今」の誤差 rms [mm]', lambda r: r['metrics']['now_err_rms'], 1000, 1),
    ('自己位置 CPU [%]', lambda r: r['cpu']['groups'].get('localization', {}).get('cpu_percent'), 1, 1),
    ('追従 CPU [%]', lambda r: r['cpu']['groups'].get('navigation', {}).get('cpu_percent'), 1, 1),
    ('自己位置 1 スキャンあたり [ms]', lambda r: 1000 * r['cpu']['groups'].get('localization', {}).get('cpu_s', math.nan)
        / max(r['metrics']['scan_count'], 1), 1, 2),
    ('自己位置 メモリ [MB]', lambda r: r['cpu']['groups'].get('localization', {}).get('rss_mb'), 1, 0),
]


def mean_std(values):
    v = [x for x in values if x is not None and not (isinstance(x, float) and math.isnan(x))]
    if not v:
        return math.nan, math.nan, 0
    m = sum(v) / len(v)
    s = math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1)) if len(v) > 1 else 0.0
    return m, s, len(v)


def fmt(m, s, scale, digits):
    if math.isnan(m):
        return '-'
    return f'{m * scale:.{digits}f} ± {s * scale:.{digits}f}'


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('runs', nargs='+', help='runs.jsonl (複数可)')
    parser.add_argument('--by', choices=['scenario', 'variant'], default='scenario',
                        help='表を分ける単位 (既定: 状況ごとに 1 表、行が構成)')
    args = parser.parse_args()

    runs = []
    for path in args.runs:
        with open(path) as f:
            runs.extend(json.loads(line) for line in f if line.strip())

    groups = OrderedDict()
    for r in runs:
        outer, inner = (r['scenario'], r['variant']) if args.by == 'scenario' else (r['variant'], r['scenario'])
        groups.setdefault(outer, OrderedDict()).setdefault(inner, []).append(r)

    for outer, rows in groups.items():
        print(f'### {outer}\n')
        header = ['構成' if args.by == 'scenario' else '状況', '成功'] + [c[0] for c in COLUMNS]
        print('| ' + ' | '.join(header) + ' |')
        print('| ' + ' | '.join(['---'] * len(header)) + ' |')
        for inner, rs in rows.items():
            ok = sum(1 for r in rs if r['ok'])
            cells = [inner, f'{ok}/{len(rs)}']
            for _, get, scale, digits in COLUMNS:
                values = []
                for r in rs:
                    try:
                        values.append(get(r))
                    except (KeyError, TypeError, ZeroDivisionError):
                        values.append(None)
                m, s, _ = mean_std(values)
                cells.append(fmt(m, s, scale, digits))
            print('| ' + ' | '.join(cells) + ' |')
        errors = [f"{r['variant']} {r['scenario']} seed={r['seed']}: {r['error']}" for rs in rows.values() for r in rs
                  if r['error']]
        if errors:
            print('\n失敗:\n')
            for e in errors:
                print(f'- {e}')
        print()


if __name__ == '__main__':
    main()
