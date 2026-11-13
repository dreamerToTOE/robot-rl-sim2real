"""Synthetic simulation logs only; no simulator, vendor model, or hardware."""

import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from robot_rl_sim2real.sim_validation import summarize_samples, validate_sample


LIMITS = {
    "parameter_class": "SIM_ONLY", "joint_names": ["a", "b"],
    "lower_rad": [-1, -1], "upper_rad": [1, 1],
    "max_velocity_rad_s": [0.5, 0.5], "effort_limit_nm": [2, 2],
    "fk_position_tolerance_m": 1e-4, "fk_orientation_tolerance_rad": 1e-3,
}


def sample(time=0, actual=0):
    return {"time_s": time, "phase": "a_positive", "joint_names": ["a", "b"],
            "q_target_rad": [0.01, 0], "q_actual_rad": [actual, 0],
            "dq_actual_rad_s": [0.01, 0], "joint_effort_nm": [0.2, 0],
            "effort_source": "SIM_ONLY actuator estimate", "fk_position_error_m": 1e-6,
            "fk_orientation_error_rad": 1e-5, "collision_detected": False,
            "collision_source": "synthetic contact query"}


class TestSimulationEvidence(unittest.TestCase):
    def baseline(self):
        row = sample(0)
        row.update(phase="hold", q_target_rad=[0, 0])
        return row

    def test_normal_step_direction_error_and_fk(self):
        result = summarize_samples([self.baseline(), sample(0.1), sample(1, 0.009), sample(2, 0.01)], LIMITS)
        self.assertEqual(result["stop_reasons"], [])
        joint = result["phases"]["a_positive"]["joints"]["a"]
        self.assertTrue(joint["direction_matches"])
        self.assertAlmostEqual(joint["max_abs_error_rad"], 0.01)
        self.assertEqual(joint["final_error_rad"], 0)
        self.assertEqual(joint["error_sign_changes"], 0)
        self.assertEqual(joint["steady_max_abs_error_rad"], 0)
        self.assertAlmostEqual(joint["settling_max_abs_error_rad"], 0.01)
        self.assertIsNone(result["phases"]["hold"]["joints"]["a"]["direction_matches"])

    def test_mapping_nonfinite_missing_contact_and_source_fail_closed(self):
        for field, value, reason in (
                ("joint_names", ["b", "a"], "joint_mapping_mismatch"),
                ("q_actual_rad", [float("nan"), 0], "invalid_q_actual_rad"),
                ("q_actual_rad", [0], "invalid_q_actual_rad"),
                ("q_actual_rad", [True, 0], "invalid_q_actual_rad"),
                ("joint_effort_nm", [float("inf"), 0], "invalid_joint_effort_nm"),
                ("collision_detected", None, "missing_collision_check"),
                ("effort_source", "", "missing_effort_source")):
            row = sample()
            row[field] = value
            with self.subTest(field=field, value=value):
                self.assertIn(reason, validate_sample(row, LIMITS))

    def test_position_velocity_collision_and_fk_gates(self):
        row = sample()
        row.update(q_target_rad=[1.1, 0], q_actual_rad=[-1.1, 0],
                   dq_actual_rad_s=[0.6, 0], joint_effort_nm=[2.1, 0], collision_detected=True,
                   fk_position_error_m=0.1, fk_orientation_error_rad=0.1)
        reasons = validate_sample(row, LIMITS)
        self.assertEqual(set(reasons), {"q_target_rad_outside_limits", "q_actual_rad_outside_limits",
                                      "abnormal_velocity", "collision_detected",
                                      "fk_position_mismatch", "fk_orientation_mismatch",
                                      "applied_effort_exceeds_limit"})

    def test_saturation_crossings_and_wrong_direction_are_descriptive(self):
        rows = [self.baseline(), sample(0.1), sample(1, 0.02), sample(2, -0.01)]
        rows[2]["joint_effort_nm"] = [2, 0]
        joint = summarize_samples(rows, LIMITS)["phases"]["a_positive"]["joints"]["a"]
        self.assertEqual(joint["error_sign_changes"], 2)
        self.assertAlmostEqual(joint["effort_limit_fraction"], 1 / 3)
        self.assertFalse(joint["direction_matches"])

    def test_empty_time_and_target_change_fail_closed(self):
        self.assertEqual(summarize_samples([], LIMITS)["stop_reasons"], [{"reason": "no_samples"}])
        result = summarize_samples([sample(1), sample(1)], LIMITS)
        self.assertEqual(result["stop_reasons"], [{"sample": 1, "reason": "nonmonotonic_timestamp"}])
        rows = [sample(0), sample(1)]
        rows[1]["q_target_rad"] = [0.02, 0]
        self.assertEqual(summarize_samples(rows, LIMITS)["stop_reasons"],
                         [{"phase": "a_positive", "reason": "target_changed_within_phase"}])

    def test_unchanged_target_has_no_direction_and_multi_axis_change_is_rejected(self):
        rows = [self.baseline(), sample(0.1), sample(1, 0.009), sample(2, 0.008)]
        rows[-1]["phase"] = "same_target"
        result = summarize_samples(rows, LIMITS)
        self.assertIsNone(result["phases"]["same_target"]["joints"]["a"]["direction_matches"])
        rows[-1]["q_target_rad"] = [0, 0.01]
        self.assertIn({"phase": "same_target", "reason": "multiple_joint_targets_changed"},
                      summarize_samples(rows, LIMITS)["stop_reasons"])

    def test_invalid_or_unclassified_thresholds_are_rejected(self):
        for key, value in (("parameter_class", "HARDWARE"), ("joint_names", ["a", "a"]),
                           ("max_velocity_rad_s", [0, 1]), ("lower_rad", [2, 0]),
                           ("fk_position_tolerance_m", float("nan"))):
            limits = copy.deepcopy(LIMITS)
            limits[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_sample(sample(), limits)


if __name__ == "__main__":
    unittest.main()
