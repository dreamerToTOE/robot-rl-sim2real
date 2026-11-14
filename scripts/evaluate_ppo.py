#!/usr/bin/env python3
"""Independent SIM_ONLY evaluation; deterministic actor and shared action interface."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src'))
sys.path.insert(0, str(REPO / 'scripts'))


def finite_json(value):
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def episode_metrics(records):
    """Failures remain in the denominator; short failed circles are not successes."""
    result = evaluate_circle_success(records)
    position = np.array([r['position_error_m'] for r in records])
    actions = np.array([r['applied_action'] for r in records])
    reasons = sorted({reason for r in records for reason in r['termination_reasons']})
    result.update({
        'steps': len(records), 'duration_s': records[-1]['episode_time_s'],
        'rmse_m': float(np.sqrt(np.mean(position**2))),
        'p95_m': float(np.percentile(position, 95)), 'max_error_m': float(position.max()),
        'max_velocity_rad_s': float(np.max(np.abs([r['dq_rad_s'] for r in records]))),
        'max_acceleration_rad_s2': float(np.max(np.abs([r['ddq_rad_s2'] for r in records]))),
        'rms_acceleration_rad_s2': float(np.sqrt(np.mean(np.square([r['ddq_rad_s2'] for r in records])))),
        'max_effort_nm': float(np.max(np.abs([r['command_effort_nm'] for r in records]))),
        'action_saturation_fraction': float(np.mean(np.abs(actions) >= 1 - 1e-6)),
        'target_clip_step_fraction': float(np.mean([r['position_clipped'] for r in records])),
        'action_change_rms': float(np.sqrt(np.mean(np.diff(actions, axis=0)**2))) if len(actions) > 1 else 0.0,
        'reward_components': {key: float(sum(r['reward'][key] for r in records))
                              for key in ('tracking', 'action', 'action_change', 'total')},
        'termination_reasons': reasons,
        'collision': bool(set(reasons) & {'geometry_collision', 'physx_contact', 'collision_unknown'}),
    })
    return result


def aggregate_episodes(episodes):
    if not episodes:
        raise ValueError('no evaluated episodes')
    n = len(episodes)
    result = {'episodes': n, 'success_rate': sum(e['success'] for e in episodes) / n,
              'complete_circle_rate': sum(e['complete_circle'] for e in episodes) / n,
              'abnormal_termination_rate': sum(e['terminated'] for e in episodes) / n,
              'collision_rate': sum(e['collision'] for e in episodes) / n,
              'mean_rmse_m': float(np.mean([e['rmse_m'] for e in episodes])),
              'median_rmse_m': float(np.median([e['rmse_m'] for e in episodes])),
              'mean_p95_m': float(np.mean([e['p95_m'] for e in episodes])),
              'maximum_error_m': max(e['max_error_m'] for e in episodes),
              'mean_episode_duration_s': float(np.mean([e['duration_s'] for e in episodes])),
              'mean_return': float(np.mean([e['reward_components']['total'] for e in episodes])),
              'termination_reason_counts': dict(Counter(r for e in episodes for r in e['termination_reasons'])),
              'metric_scope': 'episode metrics include failures; circle metrics may be unavailable for early failures',
              'individual_results': episodes}
    for key in ('max_velocity_rad_s', 'max_acceleration_rad_s2', 'max_effort_nm'):
        result[key] = max(e[key] for e in episodes)
    for key in ('rms_acceleration_rad_s2', 'action_saturation_fraction', 'target_clip_step_fraction', 'action_change_rms'):
        result['mean_' + key] = float(np.mean([e[key] for e in episodes]))
    circles = [e for e in episodes if math.isfinite(e['circle_rmse_m'])]
    result['circle_metric_episode_count'] = len(circles)
    for key in ('circle_rmse_m', 'circle_p95_m', 'circle_max_m'):
        result['mean_' + key] = float(np.mean([e[key] for e in circles])) if circles else None
    return result


def run_evaluation(env, policy_fn, seeds, output, policy_name):
    """Run one episode per seed without learning; policy_fn(obs, env) emits bounded actions.

    Reset uses the existing rejection sampler with a fixed RNG per active slot.
    Auto-reset episodes in finished slots are stepped but excluded from evidence.
    """
    import torch

    seeds = list(map(int, seeds))
    if not seeds or len(set(seeds)) != len(seeds):
        raise ValueError('evaluation seeds must be nonempty and unique')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    start = time.monotonic()
    episodes, initial_states = [], []
    with (output / 'transitions.jsonl').open('w') as log:
        for offset in range(0, len(seeds), env.num_envs):
            batch = seeds[offset:offset + env.num_envs]
            # Spare slots never enter metrics; reset distribution is otherwise identical.
            env.rngs = [np.random.default_rng(batch[i] if i < len(batch) else batch[0] + i)
                        for i in range(env.num_envs)]
            obs, _ = env.reset()
            initial_q = env._state()[0].copy()
            for i, seed in enumerate(batch):
                initial_states.append({'seed': seed, 'q_rad': initial_q[i].tolist()})
            active = [True] * len(batch)
            records = [[] for _ in batch]
            env.evaluation_seeds = batch
            # PhysX-backed DirectRLEnv tensors are written during reset/step;
            # inference_mode marks them as immutable inference tensors and can
            # make the next reset fail.  Evaluation does not need autograd, so
            # no_grad preserves the normal tensor lifecycle.
            with torch.no_grad():
                for _ in range(env.max_episode_length):
                    action = policy_fn(obs['policy'], env)
                    action = torch.as_tensor(action, dtype=torch.float32, device=env.device)
                    if action.shape != (env.num_envs, 6) or not torch.isfinite(action).all() or (action.abs() > 1+1e-6).any():
                        raise ValueError('evaluation policy violated bounded finite action contract')
                    obs, reward, terminated, truncated, _ = env.step(action)
                    if not torch.isfinite(reward).all() or not torch.isfinite(obs['policy']).all():
                        raise RuntimeError('nonfinite evaluation state or reward')
                    for i, seed in enumerate(batch):
                        if not active[i]:
                            continue
                        row = dict(env.last_transition[i])
                        row.update({'episode_index': offset+i, 'seed': seed, 'policy': policy_name})
                        records[i].append(row)
                        log.write(json.dumps(finite_json(row), allow_nan=False) + '\n')
                        if bool(terminated[i]) or bool(truncated[i]):
                            active[i] = False
                    if not any(active):
                        break
            if any(active):
                raise RuntimeError('evaluation reached horizon without a terminal signal')
            for i, seed in enumerate(batch):
                episodes.append({'seed': seed, 'episode_index': offset+i, **episode_metrics(records[i])})
            log.flush()
    summary = aggregate_episodes(episodes)
    summary.update({'parameter_class': 'SIM_ONLY', 'policy': policy_name,
                    'controller_mode': env.task['controller_mode'], 'seeds': seeds,
                    'wall_time_s': time.monotonic()-start, 'initial_states': initial_states,
                    'success_criteria': episodes[0]['criterion'], 'weights_updated': False})
    write_json(output / 'summary.json', finite_json(summary))
    return summary


def plot_episodes(run_directory):
    """Representative selection is first success and first failure, never best error."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    root = Path(run_directory)
    summary = json.loads((root / 'summary.json').read_text())
    selected = {}
    for row in summary['individual_results']:
        selected.setdefault('success' if row['success'] else 'failure', row['episode_index'])
    rows = {idx: [] for idx in selected.values()}
    with (root / 'transitions.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['episode_index'] in rows:
                rows[row['episode_index']].append(row)
    for outcome, index in selected.items():
        trace = rows[index]
        desired = np.array([r['reference_position_m'] for r in trace])
        actual = np.array([r['position_m'] for r in trace])
        times = np.array([r['episode_time_s'] for r in trace])
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        axes[0].plot(desired[:, 0]*1000, desired[:, 1]*1000, label='Reference')
        axes[0].plot(actual[:, 0]*1000, actual[:, 1]*1000, label='Actual FK')
        axes[0].set(xlabel='X (mm)', ylabel='Y (mm)', title='XY trajectory')
        axes[0].axis('equal'); axes[0].legend()
        axes[1].plot(times, np.array([r['position_error_m'] for r in trace])*1000)
        axes[1].axhline(5, color='tab:orange', linestyle='--', label='5 mm')
        axes[1].set(xlabel='Episode time (s)', ylabel='Position error (mm)', title='Error over time')
        axes[1].legend()
        for axis in axes:
            axis.grid(True, alpha=.3)
        fig.suptitle(f"SIM_ONLY | {summary['policy']} | {summary['controller_mode']} | first {outcome}, episode {index}")
        fig.tight_layout()
        fig.savefig(root / f'{outcome}_trajectory.png', dpi=150)
        plt.close(fig)


def main():
    global evaluate_circle_success, digest, write_json
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--usd', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=REPO / 'configs/b2_s1_reward_sim_only.json')
    parser.add_argument('--trajectory', type=Path, default=REPO / 'runs/b1_s1/plan_005/trajectory.jsonl')
    parser.add_argument('--actor', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--policies', nargs='+', choices=('ppo', 'zero', 'random', 'ik', 'ik_gravity'), default=['ppo'])
    parser.add_argument('--episodes', type=int, default=100)
    parser.add_argument('--seed-base', type=int, default=83000000)
    parser.add_argument('--num_envs', type=int, default=4)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.episodes <= 0 or args.seed_base < 83000000:
        parser.error('final test requires positive episodes and reserved seed-base >= 83000000')
    if 'ppo' in args.policies and args.actor is None:
        parser.error('--actor is required for PPO evaluation')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'command.json').write_text(json.dumps(
        {'argv': sys.argv, 'parameter_class': 'SIM_ONLY', 'split': 'final_test'}, indent=2) + '\n')
    app = AppLauncher(args).app
    env = None
    try:
        # Import SciPy-backed project modules only after Kit has initialized;
        # otherwise Isaac Sim's bundled scipy and the pinned venv scipy mix.
        import torch
        from robot_rl_sim2real.circle_rl import evaluate_circle_success
        from validate_sim_robot import digest, write_json
        from robot_rl_sim2real.isaac_circle_env import CircleEnvCfg, CircleTrackingEnv
        task = json.loads(args.config.read_text())
        seeds = [args.seed_base + i*10007 for i in range(args.episodes)]
        actor = torch.jit.load(str(args.actor), map_location='cpu').eval() if args.actor else None
        trajectory = None
        if any(p.startswith('ik') for p in args.policies):
            expected = json.loads((REPO / task['baseline_summary']).read_text())['trajectory_sha256']
            if digest(args.trajectory) != expected:
                raise ValueError('unreviewed IK diagnostic trajectory')
            trajectory = [json.loads(line) for line in args.trajectory.read_text().splitlines()]
        summaries = {}
        for controller in ('pd', 'pd_gravity'):
            policies = [p for p in args.policies if (p == 'ik_gravity') == (controller == 'pd_gravity')]
            if not policies:
                continue
            cfg = CircleEnvCfg(seed=seeds[0])
            cfg.scene.num_envs = args.num_envs
            cfg.scene.env_spacing = task['environment_spacing_m']
            cfg.decimation = task['decimation']; cfg.sim.dt = task['physics_dt_s']
            cfg.episode_length_s = task['episode_duration_s']
            env = CircleTrackingEnv(cfg, task | {'controller_mode': controller}, args.urdf, args.usd, args.output / controller)
            for name in policies:
                randoms = {}
                def policy(obs, current_env):
                    if name == 'ppo':
                        return actor(obs.cpu())
                    if name == 'zero':
                        return torch.zeros((current_env.num_envs, 6))
                    if name == 'random':
                        for seed in current_env.evaluation_seeds:
                            randoms.setdefault(seed, np.random.default_rng(seed + 17))
                        return np.array([randoms[current_env.evaluation_seeds[i]].uniform(-1, 1, 6)
                                         if i < len(current_env.evaluation_seeds) else np.zeros(6)
                                         for i in range(current_env.num_envs)])
                    times = np.array([r['time_s'] for r in trajectory])
                    qs = np.array([r['q_target_rad'] for r in trajectory])
                    future = current_env.episode_length_buf.cpu().numpy()*current_env.step_dt + 4 + current_env.step_dt
                    desired = np.array([np.interp(np.minimum(14, future), times, qs[:, j]) for j in range(6)]).T
                    return np.clip((desired-current_env.target)/task['action_scale_rad'], -1, 1)
                summaries[name] = run_evaluation(env, policy, seeds, args.output / name, name)
                plot_episodes(args.output / name)
            env.close(); env = None
        first = next(iter(summaries.values()))['initial_states']
        for summary in summaries.values():
            if summary['initial_states'] != first:
                raise RuntimeError('baseline initial-state set differs')
        write_json(args.output / 'comparison.json', finite_json({
            'parameter_class': 'SIM_ONLY', 'split': 'final_test', 'task_sha256': digest(args.config),
            'actor_sha256': digest(args.actor) if args.actor else None,
            'source_sha256': digest(__file__), 'same_initial_states': True,
            'summaries': summaries}))
    finally:
        if env is not None:
            env.close()
        app.close()


if __name__ == '__main__':
    main()
