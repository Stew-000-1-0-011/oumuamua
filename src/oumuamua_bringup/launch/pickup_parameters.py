"""Resolve shared CAD and per-gripper lift settings for task_commander.

No ROS imports: the same loader can be checked offline and used by launch.
"""
import math
from pathlib import Path

import yaml


def load_polygon(path, yaw):
    with open(path) as f:
        doc = yaml.safe_load(f)
    points = doc['footprint']
    if isinstance(points, str):
        points = yaml.safe_load(points)
    try:
        points = [(float(x), float(y)) for x, y in points]
    except (TypeError, ValueError) as e:
        raise ValueError('footprint must contain [x,y] pairs') from e
    if len(points) < 3 or not math.isfinite(yaw) or not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('footprint and rotation must be finite')
    c, s = math.cos(yaw), math.sin(yaw)
    return [v for x, y in points for v in (c*x-s*y, s*x+c*y)]


def pickup_parameters(task_file, gripper_file, raised_file, lowered_file, require_feedback=True, field_file=None):
    with open(task_file) as f:
        task = yaml.safe_load(f)['task_commander']['ros__parameters']
    with open(gripper_file) as f:
        grippers = yaml.safe_load(f)
    with open(field_file or Path(task_file).with_name('field.yaml')) as f:
        field = yaml.safe_load(f)['/**']['ros__parameters']['field']
    obstacle_height = max(float(field['note_size']), float(field.get('physical_wall_height', .09)))
    vertical_margin = float(task['min_clearance'])
    if not math.isfinite(obstacle_height) or obstacle_height <= 0 or not math.isfinite(vertical_margin) or vertical_margin < 0:
        raise ValueError('invalid pickup obstacle height or clearance')
    # Only these indices are used by PICK; shelf-row heights remain untouched.
    indices = [task['lift'][name] for name in ('pick', 'clear', 'carry')]
    heights = []
    tolerances = []
    common = grippers.get('/**', {}).get('ros__parameters', {}).get('lift', {})
    for name in task['grippers']['names']:
        lift = dict(common)
        lift.update(grippers.get(name, {}).get('ros__parameters', {}).get('lift', {}))
        positions = lift.get('positions', [])
        if any(isinstance(i, bool) or not isinstance(i, int) or i < 0 or i >= len(positions) for i in indices):
            raise ValueError(f'{name}: pick/clear/carry index is outside lift.positions')
        selected = [float(positions[i]) for i in indices]
        tolerance = float(lift.get('tolerance', .005))
        if not all(math.isfinite(v) for v in selected) or not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError(f'{name}: invalid lift heights or tolerance')
        if selected[1] <= selected[0] or selected[2]-selected[0] < obstacle_height+vertical_margin:
            raise ValueError(f'{name}: clear must exceed pick; carry must clear notes/walls plus margin')
        heights.extend(selected)
        tolerances.append(tolerance)
    yaw = float(task['pick'].get('footprint_yaw', math.pi/2))
    return {
        'pick.raised_footprint': load_polygon(raised_file, yaw),
        'pick.lowered_footprint': load_polygon(lowered_file, yaw),
        'pick.lift_heights': heights,
        'pick.lift_tolerances': tolerances,
        'pick.require_feedback': bool(require_feedback),
    }
