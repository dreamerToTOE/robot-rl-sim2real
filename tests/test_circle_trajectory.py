"""Synthetic kinematics and phase transitions; no private model or simulator."""

from pathlib import Path
import math
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
try:
    import numpy as np
    import scipy
except ModuleNotFoundError:
    dependencies_available = False
else:
    dependencies_available = True
    from compare_model_fk import parse_urdf
    from run_circle_trajectory import summary
    from robot_rl_sim2real.circle_trajectory import (
        circle_phase_count, circle_position, numerical_jacobian, phase_ramp_integral,
        quintic_blend, quintic_joint_path, rotation_vector, solve_fixed_orientation,
    )


@unittest.skipUnless(dependencies_available, "Optional NumPy/SciPy unavailable")
class TestCircleTrajectory(unittest.TestCase):
    def model(self):
        axes = ("0 0 1", "0 1 0", "1 0 0", "0 1 0", "1 0 0", "0 0 1")
        origins = ("0 0 0.1", "0.1 0 0.1", "0.1 0.1 0", "0.1 0 0.05", "0 0.1 0.05", "0.1 0 0")
        xml = '<robot name="synthetic"><link name="base"/>'
        for index, (axis, origin) in enumerate(zip(axes, origins), 1):
            parent = "base" if index == 1 else f"link{index-1}"
            xml += (f'<link name="link{index}"/><joint name="joint{index}" type="revolute">'
                    f'<parent link="{parent}"/><child link="link{index}"/>'
                    f'<origin xyz="{origin}"/><axis xyz="{axis}"/>'
                    '<limit lower="-2" upper="2" effort="0" velocity="0"/></joint>')
        xml += '</robot>'
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "synthetic.urdf"
        path.write_text(xml)
        return parse_urdf(path)

    def test_constant_circle_and_smooth_velocity_ramp_boundaries(self):
        period, ramp = 8.0, 1.0
        omega = 2 * math.pi / period
        self.assertAlmostEqual(-omega*ramp/2 + omega*ramp*phase_ramp_integral(1), 0)
        self.assertAlmostEqual(phase_ramp_integral(0), 0)
        self.assertAlmostEqual(phase_ramp_integral(1), 0.5)
        h = 1e-5
        for s in (0.2, 0.5, 0.8):
            slope = (phase_ramp_integral(s+h) - phase_ramp_integral(s-h)) / (2*h)
            self.assertAlmostEqual(slope, quintic_blend(s), places=8)
        self.assertAlmostEqual(quintic_blend(0), 0)
        self.assertAlmostEqual(quintic_blend(1), 1)
        a = circle_position([1, 2, 3], 0.01, 0)
        b = circle_position([1, 2, 3], 0.01, omega*period)
        np.testing.assert_allclose(a, b, atol=1e-14)

    def test_ingress_starts_and_stops_smoothly(self):
        start, end = np.zeros(6), np.ones(6)*0.2
        path = quintic_joint_path(start, end, 1001)
        np.testing.assert_array_equal(path[0], start)
        np.testing.assert_allclose(path[-1], end)
        self.assertLess(float(np.max(np.abs(path[1]-path[0]))), 1e-8)
        self.assertLess(float(np.max(np.abs(path[-1]-path[-2]))), 1e-8)
        with self.assertRaises(ValueError):
            circle_phase_count(1.0, 0.3)
        with self.assertRaises(ValueError):
            circle_position([0, 0, 0], float("nan"), 0)

    def test_bounded_ik_reconstructs_known_pose_and_rejects_unreachable_target(self):
        model = self.model()
        names = [joint.name for joint in model.moving]
        target_q = np.array([0.1, 0.15, -0.12, 0.2, 0.15, -0.2])
        target = np.array(model.fk(dict(zip(names, target_q)))['link6'])
        lower, upper = np.full(6, -2.0), np.full(6, 2.0)
        solved, residual = solve_fixed_orientation(model, names, target[:3, 3], target[:3, :3],
                                                  target_q+0.01, lower, upper)
        np.testing.assert_allclose(solved, target_q, atol=1e-7)
        self.assertLess(residual, 1e-8)
        with self.assertRaises(ValueError):
            solve_fixed_orientation(model, names, np.array([10, 10, 10]), target[:3, :3],
                                    target_q, lower, upper)

    def test_numerical_jacobian_predicts_local_pose_change(self):
        model = self.model()
        names = [joint.name for joint in model.moving]
        q = np.array([0.1, 0.15, -0.12, 0.2, 0.15, -0.2])
        delta = np.array([1, -2, 3, -4, 5, -6]) * 1e-7
        first = np.array(model.fk(dict(zip(names, q)))['link6'])
        second = np.array(model.fk(dict(zip(names, q+delta)))['link6'])
        actual = np.r_[second[:3, 3]-first[:3, 3], rotation_vector(first[:3, :3].T @ second[:3, :3])]
        np.testing.assert_allclose(numerical_jacobian(model, names, q) @ delta, actual, atol=1e-11)

    def test_summary_known_circle_bias_and_early_stop(self):
        names = [f"joint{i}" for i in range(1, 7)]
        cfg = {"circle_center_m": [0, 0, 0], "circle_radius_m": 0.01,
               "controller": {"effort_limit_nm": 20}}
        rows = []
        for i in range(5):
            phase = i * math.pi / 2
            rows.append({"phase": "circle", "circle_phase_rad": phase, "time_s": i/4,
                         "q_target_rad": [0]*6, "q_actual_rad": [0, 0, 0.005, 0, 0, 0],
                         "tracking_error_rad": [0, 0, -0.005, 0, 0, 0],
                         "dq_actual_rad_s": [0]*6, "ddq_actual_rad_s2": [0]*6,
                         "joint_effort_nm": [0]*6, "position_error_m": 0.002,
                         "orientation_error_rad": 0, "fk_position_error_m": 0,
                         "fk_orientation_error_rad": 0, "collision_detected": False,
                         "limit_violation": False, "minimum_aabb_distance_lower_bound_m": 0,
                         "actual_link6_position_m": [0.01*math.cos(phase), 0.01*math.sin(phase), 0.002]})
        result = summary(rows, cfg, names, [], [])
        self.assertAlmostEqual(result["tracking"]["rmse_position_m"], 0.002)
        self.assertAlmostEqual(result["tracking"]["p95_position_m"], 0.002)
        self.assertTrue(result["circle_completion"]["completed_one_circle"])
        self.assertEqual(next(iter(result["tracking"]["joint_error_max_rad"])), "joint3")
        rows[0]["phase"] = "initial_hold"
        stopped = summary(rows[:1], cfg, names, [], ["collision"])
        self.assertEqual(stopped["status"], "BLOCKED")
        self.assertIsNone(stopped["tracking"])


if __name__ == "__main__":
    unittest.main()
