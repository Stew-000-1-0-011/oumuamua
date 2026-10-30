"""Geometry/CLI regression tests; run without ROS using unittest."""
import importlib.util
import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'make_path.py'
spec = importlib.util.spec_from_file_location('make_path', SCRIPT)
mp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mp)


class GeometryTest(unittest.TestCase):
    # These fail if the source axes are ignored, or file-encoded polygons are dropped.
    def test_external_string_polygon_is_rotated_into_base_link(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'body.yaml'
            p.write_text('footprint: "[[0, 1], [1, 0], [-1, 0]]"\n')
            points = mp.load_footprint(p, math.pi / 2)
            for point, expected in zip(points, [(-1, 0), (0, 1), (0, -1)]):
                self.assertAlmostEqual(point[0], expected[0])
                self.assertAlmostEqual(point[1], expected[1])

    def test_malformed_polygon_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'body.yaml'
            for shape in ['[[0, 0], [1, 1]]', '[[0, 0], [1, 0], [.nan, 1]]']:
                p.write_text('footprint: ' + shape)
                with self.assertRaises(ValueError):
                    mp.load_footprint(p)

    def test_rectangle_inside_body_is_collision(self):
        poly = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        self.assertEqual(mp.polygon_clearance(poly, None, [[-.1, -.1, .1, .1]]), 0)

    def test_edge_crossing_without_contained_vertices_is_collision(self):
        poly = [(-1, -.1), (1, -.1), (1, .1), (-1, .1)]
        self.assertEqual(mp.polygon_clearance(poly, None, [[-.1, -1, .1, 1]]), 0)

    def test_polygon_distance_and_bounds(self):
        poly = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        self.assertAlmostEqual(mp.polygon_clearance(poly, None, [[2, 2, 3, 3]]), math.sqrt(2))
        self.assertAlmostEqual(mp.polygon_clearance(poly, [-.9, -2, 2, 2], []), -.1)

    def test_concavity_is_preserved(self):
        poly = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
        self.assertAlmostEqual(mp.polygon_clearance(poly, None, [[1.2, 1.2, 1.4, 1.4]]), .2)

    def test_translation_between_safe_vertices_is_checked(self):
        path = [(-1, 0, 0), (1, 0, 0)]
        distance, _ = mp.validate_path(path, {'robot_radius': .01}, None, [[-.05, -.05, .05, .05]])
        self.assertLessEqual(distance, 0)

    def test_rotation_between_safe_vertices_is_checked(self):
        route = {'footprint': [(-1, -.01), (1, -.01), (1, .01), (-1, .01)]}
        # Horizontal endpoints are safe; at yaw=pi/2 the long arm strikes the box.
        distance, _ = mp.validate_path([(0, 0, 0), (.001, 0, math.pi)], route, None, [[-.05, .8, .05, .9]])
        self.assertLessEqual(distance, 0)

    def test_circle_uses_true_radius(self):
        distance, _ = mp.validate_path([(0, 0, 0), (.01, 0, 0)], {'robot_radius': 1}, None, [[1.03, -.01, 1.04, .01]])
        self.assertLessEqual(distance, .02 + 1e-10)
        self.assertGreater(distance, .017)


class CliTest(unittest.TestCase):
    def run_route(self, route, extra_files=None):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            for name, content in (extra_files or {}).items():
                (d / name).write_text(content)
            (d / 'route.yaml').write_text(yaml.safe_dump(route))
            output = d / 'path.csv'
            output.write_text('previous valid path\n')
            result = subprocess.run([sys.executable, str(SCRIPT), str(d / 'route.yaml'), str(output)], capture_output=True, text=True)
            return result, output.read_text()

    def test_low_endpoint_failure_preserves_existing_output(self):
        result, output = self.run_route({
            'corners': [[0, 0], [.2, 0]], 'robot_radius': .01,
            'endpoint_footprint_file': 'low.yaml', 'min_clearance': .04,
            'bounds': [-.5, -.5, .5, .5],
        }, {'low.yaml': 'footprint: [[-1, -.1], [1, -.1], [1, .1], [-1, .1]]'})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(output, 'previous valid path\n')

    def test_external_polygon_is_used_by_cli(self):
        result, output = self.run_route({
            'corners': [[0, 0], [.2, 0]], 'footprint_file': 'body.yaml',
            'footprint_yaw': math.pi / 2, 'bounds': [-.5, -.5, .5, .5], 'min_clearance': .04,
        }, {'body.yaml': 'footprint: "[[0, 1], [1, 0], [-1, 0]]"'})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(output, 'previous valid path\n')

    def test_field_padding_applies_before_validation(self):
        result, output = self.run_route({
            'corners': [[0, 0], [.1, 0]], 'field': 'field.yaml',
            'field_bounds_padding': .1, 'field_wall_padding': .06, 'min_clearance': .05,
        }, {'field.yaml': '/**:\n  ros__parameters:\n    field:\n      bounds: [-1, -1, 1, 1]\n      walls: [-1, .1, 1, .2]\n      shelf: [2, 2, 3, 3]\n'})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(output, 'previous valid path\n')

    def test_csv_rounding_is_validated(self):
        result, output = self.run_route({
            'corners': [[0, .040004], [.001, .040004]],
            # Unrounded sweep bound is .039504; the CSV's bound is .039500.
            # A threshold between them detects omission of CSV-rounding checks.
            'bounds': [-1, 0, 1, 1], 'min_clearance': .039502,
        })
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(output, 'previous valid path\n')


ROOT = SCRIPT.parents[3]
ROUTE = ROOT / 'src/oumuamua_bringup/config/paths/slalom.route.yaml'


class SlalomTest(unittest.TestCase):
    def read_path(self, path):
        with path.open() as f:
            return [tuple(map(float, row)) for row in csv.reader(line for line in f if not line.startswith('#') and line.strip()) if row[0] != 'x']

    def test_slalom_actual_body_has_clearance_in_both_directions(self):
        route = mp.load_route(ROUTE)
        field = mp.load_field(ROUTE.parent / route['field'], .0025, .005)
        for name in ('slalom_forward.csv', 'slalom_backward.csv'):
            path = self.read_path(ROUTE.parent / name)
            distance, _ = mp.validate_path(path, route, field['bounds'], field['obstacles'])
            # Independent estimate from the supplied raised geometry is ~125 mm;
            # this catches accidentally reverting to the old 300 mm circle.
            self.assertGreater(distance, .10)
            self.assertIn('_endpoint_footprint', route)
            low = {'footprint': route['_endpoint_footprint']}
            for pose in (path[0], path[-1]):
                self.assertGreaterEqual(mp.pose_clearance(pose, low, field['bounds'], field['obstacles']), .04)

    def test_routes_reverse_every_pose_and_agree_with_task_endpoints(self):
        forward = self.read_path(ROUTE.parent / 'slalom_forward.csv')
        backward = self.read_path(ROUTE.parent / 'slalom_backward.csv')
        self.assertEqual(forward, backward[::-1])
        task = yaml.safe_load((ROUTE.parents[1] / 'task_commander.yaml').read_text())['task_commander']['ros__parameters']['slalom']
        for pose, expected in ((forward[0], task['entrance']), (forward[-1], task['exit'])):
            for value, want in zip(pose, expected):
                self.assertAlmostEqual(value, want, delta=.000005)
        self.assertTrue(all(math.hypot(b[0]-a[0], b[1]-a[1]) > 1e-9 for a, b in zip(forward, forward[1:])))

    def test_committed_csvs_are_reproducible(self):
        with tempfile.TemporaryDirectory() as d:
            for name, extra in [('slalom_forward.csv', []), ('slalom_backward.csv', ['--reverse'])]:
                output = Path(d) / name
                result = subprocess.run([sys.executable, str(SCRIPT), str(ROUTE.relative_to(ROOT)), str(output), *extra], cwd=ROOT, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(output.read_bytes(), (ROUTE.parent / name).read_bytes())


if __name__ == '__main__':
    unittest.main()
