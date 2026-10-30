import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('pickup_parameters', ROOT / 'launch/pickup_parameters.py')
PARAMS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PARAMS)


class PickupParametersTest(unittest.TestCase):
    def load(self, task=None, gripper=None, require=True):
        return PARAMS.pickup_parameters(task or ROOT/'config/task_commander.yaml',
                                       gripper or ROOT/'config/gripper.yaml',
                                       ROOT/'config/footprints/raised.yaml',
                                       ROOT/'config/footprints/lowered.yaml', require)

    def test_source_polygon_transformed_once(self):
        data = self.load()
        self.assertAlmostEqual(data['pick.raised_footprint'][0], .083424)
        self.assertAlmostEqual(data['pick.raised_footprint'][1], -.225497)
        self.assertEqual(len(data['pick.raised_footprint']), 32)
        self.assertEqual(len(data['pick.lowered_footprint']), 206)

    def test_actual_lift_heights_are_not_duplicated_in_task_config(self):
        self.assertEqual(self.load()['pick.lift_heights'], [0., .09, .18, 0., .09, .18])
        self.assertTrue(self.load()['pick.require_feedback'])
        self.assertFalse(self.load(require=False)['pick.require_feedback'])

    def test_per_gripper_override_and_index_checks(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'gripper.yaml'
            p.write_text('/**:\n  ros__parameters:\n    lift:\n      positions: [0.0, 0.1, 0.2]\n      tolerance: 0.003\ngripper1:\n  ros__parameters:\n    lift:\n      positions: [0.01, 0.11, 0.21]\n')
            self.assertEqual(self.load(gripper=p)['pick.lift_heights'], [0.,.1,.2,.01,.11,.21])
            p.write_text('/**:\n  ros__parameters:\n    lift:\n      positions: [0.0]\n')
            with self.assertRaises(ValueError):
                self.load(gripper=p)

    def test_nonfinite_polygon_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'invalid.yaml'
            p.write_text('footprint: [[0, 0], [1, 0], [.nan, 1]]')
            with self.assertRaises(ValueError):
                PARAMS.load_polygon(p, math.pi/2)

    def test_carry_must_lift_note_above_floor_obstacles(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'gripper.yaml'
            p.write_text('/**:\n  ros__parameters:\n    lift:\n      positions: [0.0, 0.09, 0.14]\n')
            with self.assertRaises(ValueError):
                self.load(gripper=p)


if __name__ == '__main__':
    unittest.main()
