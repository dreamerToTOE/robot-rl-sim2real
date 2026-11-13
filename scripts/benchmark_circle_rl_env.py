#!/usr/bin/env python3
"""Short CPU throughput probe for the B2 circle environment; no PPO or hardware."""

import argparse
import json
from pathlib import Path
import time

REPO = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "scripts"))


def main():
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--num_envs", type=int, required=True)
    parser.add_argument("--steps", type=int, default=64)
    parser.add_argument("--controller", choices=("pd", "pd_gravity"), default="pd")
    parser.add_argument("--seed", type=int, default=20261113)
    parser.add_argument("--config", type=Path, default=REPO / "configs/b2_s0_circle_sim_only.json")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.device != "cpu" or not 1 <= args.num_envs <= 4 or args.steps <= 0:
        raise ValueError("benchmark is limited to 1..4 CPU environments and positive steps")
    args.output.mkdir(parents=True, exist_ok=False)
    task = json.loads(args.config.read_text())
    task["controller_mode"] = args.controller
    app = env = None
    try:
        app = AppLauncher(args).app
        import torch
        from robot_rl_sim2real.isaac_circle_env import CircleEnvCfg, CircleTrackingEnv

        cfg = CircleEnvCfg(seed=args.seed)
        cfg.scene.num_envs = args.num_envs
        cfg.scene.env_spacing = task["environment_spacing_m"]
        cfg.decimation = task["decimation"]
        cfg.sim.dt = task["physics_dt_s"]
        cfg.episode_length_s = task["episode_duration_s"]
        env = CircleTrackingEnv(cfg, task, args.urdf, args.usd, args.output)

        counters = {"assess": 0.0, "fk": 0.0, "collision": 0.0,
                    "pre_physics": 0.0, "dones": 0.0, "reward": 0.0,
                    "observations": 0.0, "steps": 0.0}
        counts = {key: 0 for key in counters}

        def timed_method(obj, name, key):
            original = getattr(obj, name)

            def wrapped(*call_args, **call_kwargs):
                start = time.perf_counter()
                result = original(*call_args, **call_kwargs)
                counters[key] += time.perf_counter() - start
                counts[key] += 1
                return result

            setattr(obj, name, wrapped)

        timed_method(env, "assess", "assess")
        timed_method(env.model, "fk", "fk")
        timed_method(env.geometry, "check", "collision")
        timed_method(env, "_pre_physics_step", "pre_physics")
        timed_method(env, "_get_dones", "dones")
        timed_method(env, "_get_rewards", "reward")
        timed_method(env, "_get_observations", "observations")

        env.reset(seed=args.seed)
        action = torch.zeros((args.num_envs, 6), dtype=torch.float32, device=env.device)
        # Warm-up avoids counting one-time Python/PhysX setup in the steady-state rate.
        for _ in range(4):
            env.step(action)
        for key in counters:
            counters[key] = 0.0
            counts[key] = 0
        start = time.perf_counter()
        for _ in range(args.steps):
            _, reward, terminated, truncated, _ = env.step(action)
            if not (torch.isfinite(reward).all() and torch.isfinite(terminated).all()
                    and torch.isfinite(truncated).all()):
                raise RuntimeError("non-finite benchmark output")
        elapsed = time.perf_counter() - start
        counters["steps"] = elapsed
        counts["steps"] = args.steps
        # Outer hooks are sequential. Nested assess/FK/collision hooks are reported
        # separately and intentionally excluded from this sum.
        accounted = sum(counters[k] for k in ("pre_physics", "dones", "reward", "observations"))
        result = {
            "status": "PASS",
            "parameter_class": "SIM_ONLY",
            "controller_mode": args.controller,
            "num_envs": args.num_envs,
            "steps": args.steps,
            "control_dt_s": env.step_dt,
            "steady_wall_s": elapsed,
            "control_steps_per_s": args.steps / elapsed,
            "environment_steps_per_s": args.steps * args.num_envs / elapsed,
            "physics_residual_s": max(0.0, elapsed - accounted),
            "timing_s": counters,
            "calls": counts,
            "finite_outputs": True,
            "training_started": False,
            "hardware_connected": False,
        }
        (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print("B2_S1_BENCHMARK=" + json.dumps(result, ensure_ascii=False), flush=True)
    finally:
        if env is not None:
            env.close()
        if app is not None:
            app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
