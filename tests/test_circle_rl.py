"""B2-S0 contract tests with synthetic values, no vendor files or simulator."""

import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
try:
    import numpy as np
    import scipy
except ModuleNotFoundError:
    available = False
else:
    available = True
    from robot_rl_sim2real.circle_rl import (evaluate_circle_success, map_action,
                                              observation, reference_at, reward_components)


@unittest.skipUnless(available, "Optional NumPy/SciPy unavailable")
class TestCircleRL(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT / "configs/b2_s0_circle_sim_only.json").read_text())
        self.circle = {"center_m": [0.3, 0, 0.4], "radius_m": 0.01, "period_s": 8.0}

    def test_reference_segments_circle_and_continuity(self):
        refs = [reference_at(t, self.circle, 1, 0.5) for t in (0, 1, 9, 10, 10.5)]
        self.assertEqual([r['segment'] for r in refs], ['startup', 'circle', 'circle', 'hold', 'hold'])
        self.assertAlmostEqual(refs[2]['phase_rad'] - refs[1]['phase_rad'], 2*math.pi)
        np.testing.assert_allclose(refs[1]['position_m'], refs[2]['position_m'], atol=1e-14)
        for boundary in (1, 9, 10):
            a, b = [reference_at(boundary+d, self.circle, 1, 0.5) for d in (-1e-8, 1e-8)]
            np.testing.assert_allclose(a['velocity_m_s'], b['velocity_m_s'], atol=1e-8)
            np.testing.assert_allclose(a['position_m'], b['position_m'], atol=1e-8)
        for invalid in (float('nan'), -1):
            with self.assertRaises(ValueError): reference_at(invalid, self.circle, 1, 0.5)

    def test_observation_31_units_actual_state_previous_target_and_time(self):
        q, dq, prev = np.full(6, .035), np.full(6, .1), np.full(6, .2)
        obs = observation(q, dq, np.zeros(3), np.array([.01, 0, 0]), np.zeros(3),
                          prev, q+.006, 5.25, np.zeros(6), self.cfg, self.circle)
        self.assertEqual(obs.shape, (31,))
        np.testing.assert_allclose(obs[:6], .1)
        np.testing.assert_allclose(obs[6:12], .2)
        self.assertEqual(obs[12], 1)
        np.testing.assert_allclose(obs[18:24], prev)
        np.testing.assert_allclose(obs[24:30], .5)
        self.assertEqual(obs[30], .5)
        q[0] = float('nan')
        with self.assertRaises(ValueError):
            observation(q, dq, np.zeros(3), np.zeros(3), np.zeros(3), prev, np.zeros(6), 0, np.zeros(6), self.cfg, self.circle)

    def test_same_circle_position_and_velocity_have_distinct_episode_times(self):
        start, end = [reference_at(t, self.circle, 1, .5) for t in (1, 9)]
        np.testing.assert_allclose(start['position_m'], end['position_m'], atol=1e-14)
        np.testing.assert_allclose(start['velocity_m_s'], end['velocity_m_s'], atol=1e-14)
        z=np.zeros(6); p=np.zeros(3)
        observations=[observation(z,z,p,r['position_m'],r['velocity_m_s'],z,z,t,z,self.cfg,self.circle)
                      for r,t in ((start,1),(end,9))]
        np.testing.assert_allclose(observations[0][:30],observations[1][:30],atol=1e-12)
        self.assertNotEqual(observations[0][30],observations[1][30])

    def test_action_previous_target_increment_slew_and_position_bounds(self):
        zeros = np.zeros((2, 6)); q = np.full((2, 6), .1)
        r = map_action(np.ones_like(q), zeros, q, -.11, .11, self.cfg)
        np.testing.assert_allclose(r['slewed'], .25)
        np.testing.assert_allclose(r['target'], .10075)
        r = map_action(np.ones_like(q), np.ones_like(q), q, -.101, .101, self.cfg)
        np.testing.assert_allclose(r['target'], .101)
        self.assertTrue(r['position_clipped'].all())
        q += .02
        r = map_action(zeros, zeros, q, -.2, .2, self.cfg)
        np.testing.assert_allclose(r['target'], q)

    def test_invalid_action_is_held_and_flagged(self):
        for value in (1.01, float('nan'), float('inf')):
            a = np.zeros((1, 6)); a[0, 0] = value
            r = map_action(a, np.ones_like(a)*.5, np.zeros_like(a), -1, 1, self.cfg)
            self.assertTrue(r['invalid'][0])
            np.testing.assert_array_equal(r['target'], 0)
        with self.assertRaises(ValueError):
            map_action(np.zeros(5), np.zeros(6), np.zeros(6), -1, 1, self.cfg)

    def test_reward_known_values_and_failure_not_early_exit_profit(self):
        n = 630
        r = reward_components([.005], np.ones((1, 6)), np.zeros((1, 6)), n, self.cfg)
        self.assertAlmostEqual(float(r['tracking'][0]), .03/n)
        self.assertAlmostEqual(float(r['action'][0]), -.005/n)
        self.assertAlmostEqual(float(r['action_change'][0]), -.005/n)
        self.assertAlmostEqual(float(r['total'][0]), .02/n)
        perfect = reward_components([0], np.zeros((1, 6)), np.zeros((1, 6)), n, self.cfg)
        self.assertAlmostEqual(float(perfect['total'][0]), 1.03/n)
        worst = reward_components([.1], np.ones((1, 6)), -np.ones((1, 6)), n, self.cfg)
        self.assertAlmostEqual(float(worst['total'][0]), .005/n)
        self.assertGreater(float(worst['total'][0]), 0)
        self.assertLess(float(perfect['total'][0]), float(perfect['total'][0]*n))
        with self.assertRaises(ValueError):
            reward_components([0], np.zeros((1,6)), np.zeros((1,6)), n, self.cfg | {'tracking_reward_floor': 0})

    def test_full_horizon_stationary_tracking_scores_lower_than_tracking(self):
        n=630; start = reference_at(0, self.circle, 1, .5)['position_m']
        errors = np.array([np.linalg.norm(reference_at(i/60, self.circle, 1, .5)['position_m']-start)
                           for i in range(1,n+1)])
        z=np.zeros((n,6))
        stationary=reward_components(errors,z,z,n,self.cfg)
        tracked=reward_components(np.zeros(n),z,z,n,self.cfg)
        self.assertLess(float(stationary['total'].sum()), .53)
        self.assertAlmostEqual(float(tracked['total'].sum()), 1.03)

    def test_exp_linear_reward_preserves_signal_above_five_mm(self):
        cfg = self.cfg | {'tracking_reward_mode': 'exp_linear'}
        errors = np.array([0.0, .005, .010, .020])
        result = reward_components(errors, np.zeros((4, 6)), np.zeros((4, 6)), 630, cfg)
        expected = (.03 + np.exp(-errors / .005)) / 630
        np.testing.assert_allclose(result['tracking'], expected)
        self.assertGreater(float(result['tracking'][2]), float(result['tracking'][3]))
        self.assertGreater(float(result['tracking'][1]), float(result['tracking'][2]))
        self.assertGreater(float(result['total'].min()), 0.0)

        old = reward_components(errors, np.zeros((4, 6)), np.zeros((4, 6)), 630, self.cfg)
        self.assertAlmostEqual(float(old['tracking'][2]), float(old['tracking'][3]))

    def test_reward_mode_rejects_unknown_value(self):
        with self.assertRaises(ValueError):
            reward_components([0.0], np.zeros((1, 6)), np.zeros((1, 6)), 630,
                              self.cfg | {'tracking_reward_mode': 'unknown'})

    def test_batched_actions_are_independent(self):
        q=np.zeros((2,6)); a=q.copy(); a[1,2]=1
        result=map_action(a,q,q,-1,1,self.cfg)
        np.testing.assert_array_equal(result['target'][0],0)
        self.assertAlmostEqual(result['target'][1,2],.00075)

    def test_circle_success_requires_full_circle_and_clean_timeout(self):
        records = [{'reference_segment': 'circle', 'reference_phase_rad': phase,
                    'position_error_m': .002, 'terminated': False, 'truncated': False}
                   for phase in np.linspace(0, 2*math.pi, 9)]
        records.append({'reference_segment': 'hold', 'reference_phase_rad': 2*math.pi,
                        'position_error_m': .003, 'terminated': False, 'truncated': True})
        result = evaluate_circle_success(records)
        self.assertTrue(result['success'])
        self.assertAlmostEqual(result['circle_rmse_m'], .002)
        records[-1]['truncated'] = False
        self.assertFalse(evaluate_circle_success(records)['success'])
        records[-1]['truncated'] = True
        records[3]['terminated'] = True
        self.assertFalse(evaluate_circle_success(records)['success'])

    def test_circle_success_error_limits_are_independent(self):
        records = [{'reference_segment': 'circle', 'reference_phase_rad': phase,
                    'position_error_m': .002, 'terminated': False, 'truncated': False}
                   for phase in np.linspace(0, 2*math.pi, 9)]
        records[4]['position_error_m'] = .009
        records.append({'reference_segment': 'hold', 'reference_phase_rad': 2*math.pi,
                        'position_error_m': .003, 'terminated': False, 'truncated': True})
        result = evaluate_circle_success(records)
        self.assertFalse(result['success'])
        self.assertGreater(result['circle_max_m'], result['criterion']['max_threshold_m'])


if __name__ == '__main__':
    unittest.main()
