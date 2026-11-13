import importlib.util
import math
from pathlib import Path
import sys
import unittest


@unittest.skipUnless(importlib.util.find_spec("numpy"), "optional simulation dependency numpy is unavailable")
class TestSimulatorPoseConversion(unittest.TestCase):
    def test_actor_quaternion_is_wxyz_and_normalized(self):
        scripts = Path(__file__).resolve().parents[1] / "scripts"
        sys.path.insert(0, str(scripts))
        from validate_sim_robot import transform

        matrix = transform([1, 2, 3, 2 * math.cos(math.pi / 4), 0, 0, 2 * math.sin(math.pi / 4)])
        self.assertAlmostEqual(matrix[0, 1], -1.0)
        self.assertAlmostEqual(matrix[1, 0], 1.0)
        self.assertAlmostEqual(matrix[2, 2], 1.0)
        self.assertEqual(matrix[:3, 3].tolist(), [1, 2, 3])
        with self.assertRaises(ValueError):
            transform([0, 0, 0, 0, 0, 0, 0])
