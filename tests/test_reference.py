import math
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from robot_rl_sim2real.reference import CircleXY


class TestCircleReference(unittest.TestCase):
    def setUp(self):
        self.r = CircleXY(center_m=(1.0, 2.0, 3.0), radius_m=0.2, period_s=4.0)

    def test_start_and_quarter_cycle(self):
        a = self.r.sample(0.0)
        b = self.r.sample(1.0)
        self.assertAlmostEqual(a.position_m[0], 1.2)
        self.assertAlmostEqual(a.position_m[1], 2.0)
        self.assertAlmostEqual(b.position_m[0], 1.0)
        self.assertAlmostEqual(b.position_m[1], 2.2)
        self.assertAlmostEqual(a.position_m[2], 3.0)

    def test_periodic(self):
        for k in (0.0, 0.5, 1.3, 4.0):
            a, b = self.r.sample(k), self.r.sample(k + 4.0)
            for ai, bi in zip(a.position_m, b.position_m):
                self.assertAlmostEqual(ai, bi)
            for ai, bi in zip(a.velocity_mps, b.velocity_mps):
                self.assertAlmostEqual(ai, bi)

    def test_speed_is_constant(self):
        desired = 2 * math.pi * 0.2 / 4
        for k in (0.0, 0.23, 1.0, 2.3, 3.9):
            v = self.r.sample(k).velocity_mps
            self.assertAlmostEqual(math.sqrt(sum(vi * vi for vi in v)), desired)

    def test_bad_values_rejected(self):
        with self.assertRaises(ValueError):
            CircleXY(center_m=(0, 0, 0), radius_m=0, period_s=2)
        with self.assertRaises(ValueError):
            CircleXY(center_m=(0, 0, 0), radius_m=1, period_s=-2)
        with self.assertRaises(ValueError):
            CircleXY(center_m=(0, float('nan'), 0), radius_m=1, period_s=2)
        with self.assertRaises(ValueError):
            self.r.sample(float('inf'))


if __name__ == "__main__":
    unittest.main()
