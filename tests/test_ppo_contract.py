import json
from pathlib import Path
import unittest

try:
    import torch
except ModuleNotFoundError:  # Factory venv check runs this test with torch installed.
    torch = None

import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from train_ppo import build_network


@unittest.skipIf(torch is None, "torch is provided by the Factory venv")
class PpoContractTests(unittest.TestCase):
    def test_tanh_distribution_is_bounded_and_finite(self):
        cfg = json.loads((ROOT / "configs/ppo/circle_ppo_sim_only.json").read_text())
        network = build_network(torch, cfg)
        obs = torch.zeros(8, 31)
        action, logp, value, entropy, _ = network.sample(obs)
        self.assertTrue(torch.all(action.abs() < 1))
        self.assertTrue(torch.isfinite(logp).all())
        self.assertTrue(torch.isfinite(value).all())
        self.assertTrue(torch.isfinite(entropy).all())

    def test_export_action_and_evaluation_logprob_are_finite(self):
        cfg = json.loads((ROOT / "configs/ppo/circle_ppo_sim_only.json").read_text())
        network = build_network(torch, cfg)
        obs = torch.randn(4, 31)
        action, _, _, _, _ = network.sample(obs)
        logp, entropy, value = network.evaluate(obs, action)
        self.assertTrue(torch.isfinite(logp).all())
        self.assertTrue(torch.isfinite(entropy).all())
        self.assertTrue(torch.isfinite(value).all())


if __name__ == "__main__":
    unittest.main()
