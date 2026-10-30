"""Exercise the production planner with shared YAML/CAD data and draw its plans.

python src/task_commander/test/pickup_preview.py /tmp/pickup_probe output_directory
Build the probe from pickup_probe.cpp + pickup.cpp + geometry.cpp first.
"""
import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

import yaml


def main():
    root = Path(__file__).resolve().parents[3]
    config = root/'src/oumuamua_bringup/config'
    field = yaml.safe_load((config/'field.yaml').read_text())['/**']['ros__parameters']['field']
    task = yaml.safe_load((config/'task_commander.yaml').read_text())['task_commander']['ros__parameters']
    path = root/'src/oumuamua_bringup/launch/pickup_parameters.py'
    spec = importlib.util.spec_from_file_location('params', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    pick = task['pick']
    upper = module.load_polygon(config/'footprints/raised.yaml', pick['footprint_yaw'])
    lower = module.load_polygon(config/'footprints/lowered.yaml', pick['footprint_yaw'])
    obstacles = field['walls'] + field['shelf']
    contacts = pick['contact_regions']
    values = [*field['bounds'], len(obstacles)//4, *obstacles,
              len(field['shelf'])//4, *field['shelf'],
              len(field['notes_x']), *field['notes_x'], field['notes_y'], field['note_size'],
              len(upper)//2, *upper, len(lower)//2, *lower,
              len(contacts)//4, *contacts,
              pick['yaw'], pick['transit_y'], pick['standoff'], task['min_clearance'],
              pick['wall_padding'], pick['bounds_padding'], pick.get('yaw_resolution', 0.0), len(pick['yaw_offsets']), *pick['yaw_offsets'],
              *field['start_pose'], len(task['grippers']['names']), *task['grippers']['offsets']]
    result = subprocess.run([sys.argv[1]], input=' '.join(map(str, values)), capture_output=True, text=True, check=True, timeout=120)
    output = Path(sys.argv[2]); output.mkdir(parents=True, exist_ok=True)
    (output/'pickup-plans.csv').write_text(result.stdout)
    cases = []
    points = {}
    for row in csv.reader(result.stdout.splitlines()):
        scenario, box, grip = map(int, row[1:4])
        if row[0] == 'case':
            cases.append({'scenario': scenario, 'box': box, 'gripper': grip, 'valid': bool(int(row[4])), 'clearance': float(row[5])})
        else:
            points.setdefault((scenario, box, grip), []).append((row[4], *map(float, row[5:])))
    if not any(c['valid'] and c['scenario'] == 0 for c in cases):
        raise RuntimeError('no initial-layout pickup is valid; inspect approach geometry before publishing')
    for box in range(len(field['notes_x'])):
        if not any(c['valid'] and c['scenario'] == 2 and c['box'] == box for c in cases):
            raise RuntimeError(f'box {box} has no pickup candidate after right-hand boxes delivered; inspect yaw/calibration')
    (output/'pickup-results.json').write_text(json.dumps(cases, indent=2)+'\n')
    print(json.dumps(cases, indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=150)
    for scenario, ax in zip((0, 2), axes):
        b = field['bounds']
        ax.add_patch(Rectangle((b[0], b[1]), b[2]-b[0], b[3]-b[1], fill=False, ec='#444'))
        for i in range(0, len(obstacles), 4):
            a = obstacles[i:i+4]
            ax.add_patch(Rectangle((a[0], a[1]), a[2]-a[0], a[3]-a[1], fc='#444'))
        for i, x in enumerate(field['notes_x']):
            h = field['note_size']/2
            ax.add_patch(Rectangle((x-h, field['notes_y']-h), 2*h, 2*h, fc='#db924b', alpha=.45))
            ax.text(x, field['notes_y'], str(i), ha='center', va='center')
        for (iso, box, grip), poses in points.items():
            if iso != scenario:
                continue
            # Retreat retraces the approach; dash it to distinguish its direction.
            xy = [(p[1], p[2]) for p in poses if p[0] != 'retreat']
            ax.plot(*zip(*xy), label=f'box {box}, gripper {grip}', lw=1.5)
            target = poses[-2]
            ax.plot(target[1], target[2], 'o', ms=4)
            x, y, yaw = target[1:]
            c, s = math.cos(yaw), math.sin(yaw)
            outline = [(x+c*lower[i]-s*lower[i+1], y+s*lower[i]+c*lower[i+1]) for i in range(0, len(lower), 2)]
            ax.add_patch(Polygon(outline, closed=True, fc='#8a52b5', ec='#8a52b5', lw=.6, alpha=.08))
            ax.plot([x, field['notes_x'][box]], [y, field['notes_y']], '--', color='#777', lw=.8)
            ax.annotate('', (x+.12*c, y+.12*s), (x,y), arrowprops={'arrowstyle':'->','color':'#777','lw':.8})
        ax.set_title('Initial layout' if scenario == 0 else 'Boxes to the right delivered; return to start each case')
        ax.set_xlim(-2.9, -1.3); ax.set_ylim(0, 1.7); ax.set_aspect('equal'); ax.grid(alpha=.2)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
        if any(key[0] == scenario for key in points): ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(output/'pickup-plans.png')


if __name__ == '__main__':
    main()
