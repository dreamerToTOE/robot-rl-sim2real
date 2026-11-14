#!/usr/bin/env python3
"""Small standard PPO runner for the reviewed SIM_ONLY circle environment."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random
import sys
import time

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "scripts"))


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def finite(value):
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite(v) for v in value]
    if isinstance(value, np.ndarray):
        return finite(value.tolist())
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (np.integer, int)):
        return int(value)
    return value


def build_network(torch, cfg):
    import torch.nn as nn

    class ActorCritic(nn.Module):
        def __init__(self):
            super().__init__()
            def mlp(widths):
                layers = []
                for left, right in zip(widths[:-1], widths[1:]):
                    layers += [nn.Linear(left, right), nn.Tanh()]
                return nn.Sequential(*layers)
            self.actor_body = mlp([31, *cfg["actor_hidden_dims"]])
            self.critic_body = mlp([31, *cfg["critic_hidden_dims"]])
            self.mu = nn.Linear(cfg["actor_hidden_dims"][-1], 6)
            self.value = nn.Linear(cfg["critic_hidden_dims"][-1], 1)
            self.log_std = nn.Parameter(torch.full((6,), float(cfg["initial_log_std"])))
            for module in self.modules():
                if isinstance(module, nn.Linear):
                    nn.init.orthogonal_(module.weight, gain=math.sqrt(2))
                    nn.init.zeros_(module.bias)
            nn.init.orthogonal_(self.mu.weight, gain=0.01)
            nn.init.orthogonal_(self.value.weight, gain=1.0)

        def distribution(self, obs):
            mu = self.mu(self.actor_body(obs))
            std = self.log_std.exp().expand_as(mu)
            return mu, std

        def forward(self, obs):
            mu, std = self.distribution(obs)
            return mu, std, self.value(self.critic_body(obs)).squeeze(-1)

        def sample(self, obs):
            mu, std, value = self(obs)
            normal = torch.distributions.Normal(mu, std)
            latent = normal.rsample()
            action = torch.tanh(latent)
            log_prob = normal.log_prob(latent).sum(-1)
            log_prob -= torch.log(1 - action.square() + 1e-6).sum(-1)
            entropy = normal.entropy().sum(-1)
            return action, log_prob, value, entropy, latent

        def evaluate(self, obs, action):
            mu, std, value = self(obs)
            # atanh is only used for actions emitted by this policy; clamp is
            # a numerical guard and does not silently accept invalid actions.
            bounded = action.clamp(-1 + 1e-6, 1 - 1e-6)
            latent = 0.5 * (torch.log1p(bounded) - torch.log1p(-bounded))
            normal = torch.distributions.Normal(mu, std)
            log_prob = normal.log_prob(latent).sum(-1)
            log_prob -= torch.log(1 - bounded.square() + 1e-6).sum(-1)
            entropy = normal.entropy().sum(-1)
            return log_prob, entropy, value

    return ActorCritic()


def save_actor(torch, network, output: Path, cfg):
    import torch.nn as nn
    class DeterministicActor(nn.Module):
        def __init__(self, source):
            super().__init__()
            self.actor_body = source.actor_body
            self.mu = source.mu
        def forward(self, obs):
            return torch.tanh(self.mu(self.actor_body(obs)))
    actor = DeterministicActor(network).eval().cpu()
    traced = torch.jit.trace(actor, torch.zeros(1, 31))
    traced.save(str(output))
    return actor


def validation(env, network, torch, np, base_seed, device, episodes=2):
    from robot_rl_sim2real.circle_rl import evaluate_circle_success
    n = min(env.num_envs, episodes)
    seeds = [base_seed + i * 1009 for i in range(n)]
    env.rngs = [np.random.default_rng(seeds[i] if i < n else seeds[0]) for i in range(env.num_envs)]
    obs, _ = env.reset()
    active = [i < n for i in range(env.num_envs)]
    records = [[] for _ in range(n)]
    # ``inference_mode`` marks PhysX-backed tensors as inference tensors;
    # DirectRLEnv.reset writes into those tensors, so validation must use
    # no_grad rather than inference_mode.
    with torch.no_grad():
        for _ in range(env.max_episode_length):
            policy_obs = obs["policy"].to(device)
            action = network(policy_obs)[0]
            action = torch.tanh(action).cpu()
            obs, _, terminated, truncated, _ = env.step(action)
            for i in range(n):
                if active[i]:
                    records[i].append(dict(env.last_transition[i]))
                    if bool(terminated[i]) or bool(truncated[i]):
                        active[i] = False
            if not any(active[:n]):
                break
    metrics = [evaluate_circle_success(rows) for rows in records]
    rmse = [m["circle_rmse_m"] for m in metrics if math.isfinite(m["circle_rmse_m"])]
    return {
        "episodes": n,
        "success_rate": float(np.mean([m["success"] for m in metrics])),
        "mean_circle_rmse_m": float(np.mean(rmse)) if rmse else None,
        "metrics": metrics,
    }


def main():
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=REPO / "model_assets/local/vendor_extract/student_ros/src/episode1_urdf_1113/urdf/episode1_urdf_1113.urdf")
    parser.add_argument("--usd", type=Path, default=REPO / "model_assets/local/b0_r1_usd/robot.usd")
    parser.add_argument("--env-config", type=Path, default=REPO / "configs/b2_s1_reward_sim_only.json")
    parser.add_argument("--ppo-config", type=Path, default=REPO / "configs/ppo/circle_ppo_sim_only.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20261201)
    parser.add_argument("--num-envs", type=int, default=4)
    parser.add_argument("--total-steps", type=int, default=100000)
    parser.add_argument("--rollout-steps", type=int, default=128)
    parser.add_argument("--validation-every", type=int, default=10)
    parser.add_argument("--validation-episodes", type=int, default=2)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--ppo-device", default="cuda:0")
    parser.add_argument("--smoke", action="store_true")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.device != "cpu":
        raise ValueError("Isaac simulation must use --device cpu; --ppo-device selects the learner")
    # Resume is intentionally in-place so the checkpoint and metrics remain one
    # auditable run. A fresh run still refuses to overwrite anything.
    if args.output.exists():
        if args.resume is None:
            raise FileExistsError(f"output already exists: {args.output}")
    else:
        args.output.mkdir(parents=True)
    task = json.loads(args.env_config.read_text())
    ppo_cfg = json.loads(args.ppo_config.read_text())
    if args.smoke:
        args.total_steps = min(args.total_steps, args.rollout_steps * 2)
    if args.total_steps <= 0 or args.rollout_steps <= 0 or args.total_steps % args.rollout_steps:
        raise ValueError("total_steps must be a positive multiple of rollout_steps")
    if not args.urdf.exists() or not args.usd.exists():
        raise FileNotFoundError("private reviewed URDF/USD is missing")
    app = AppLauncher(args).app
    import torch
    torch.manual_seed(args.seed); np.random.seed(args.seed); random.seed(args.seed)
    if not torch.cuda.is_available() and args.ppo_device.startswith("cuda"):
        args.ppo_device = "cpu"
    device = torch.device(args.ppo_device)
    from robot_rl_sim2real.isaac_circle_env import CircleEnvCfg, CircleTrackingEnv
    cfg = CircleEnvCfg(seed=args.seed)
    cfg.scene.num_envs = args.num_envs
    cfg.scene.env_spacing = task["environment_spacing_m"]
    cfg.decimation = task["decimation"]; cfg.sim.dt = task["physics_dt_s"]
    cfg.episode_length_s = task["episode_duration_s"]
    env = CircleTrackingEnv(cfg, task, args.urdf, args.usd, args.output / "env")
    network = build_network(torch, ppo_cfg).to(device)
    optimizer = torch.optim.Adam(network.parameters(), lr=ppo_cfg["learning_rate"])
    start_update, interactions = 0, 0
    base_wall_time = 0.0
    if args.resume:
        state = torch.load(args.resume, map_location=device, weights_only=False)
        network.load_state_dict(state["model"]); optimizer.load_state_dict(state["optimizer"])
        start_update, interactions = state["update"], state["interactions"]
        base_wall_time = float(state.get("elapsed_wall_time_s", 0.0))
    obs, _ = env.reset(seed=args.seed)
    observation = obs["policy"].clone()
    total_updates = args.total_steps // args.rollout_steps
    metrics = []
    start_time = time.monotonic()
    metrics_path = args.output / "metrics.jsonl"
    metrics = []
    if args.resume and metrics_path.exists():
        metrics = [json.loads(line) for line in metrics_path.read_text().splitlines() if line.strip()]
    with metrics_path.open("a" if args.resume else "w") as metric_log:
        for update in range(start_update, total_updates):
            t0 = time.monotonic()
            obs_buf=[]; action_buf=[]; logp_buf=[]; value_buf=[]; reward_buf=[]; done_buf=[]; ent_buf=[]
            for _ in range(args.rollout_steps):
                obs_dev = observation.to(device)
                with torch.no_grad():
                    action, logp, value, entropy, _ = network.sample(obs_dev)
                action_cpu = action.cpu()
                next_obs, reward, terminated, truncated, _ = env.step(action_cpu)
                done = (terminated | truncated).to(torch.float32)
                obs_buf.append(obs_dev); action_buf.append(action); logp_buf.append(logp)
                value_buf.append(value); reward_buf.append(reward.to(device)); done_buf.append(done.to(device)); ent_buf.append(entropy)
                observation = next_obs["policy"].clone()
                interactions += env.num_envs
            with torch.no_grad():
                next_value = network(observation.to(device))[2]
            values = torch.stack(value_buf); rewards = torch.stack(reward_buf); dones = torch.stack(done_buf)
            advantages = torch.zeros_like(rewards); last = torch.zeros(env.num_envs, device=device)
            for t in reversed(range(args.rollout_steps)):
                # ``dones[t]`` is the boundary after transition t. Masking with
                # t+1 would bootstrap across an auto-reset episode.
                next_nonterminal = 1.0 - dones[t]
                next_val = next_value if t == args.rollout_steps - 1 else values[t + 1]
                delta = rewards[t] + ppo_cfg["gamma"] * next_val * next_nonterminal - values[t]
                last = delta + ppo_cfg["gamma"] * ppo_cfg["gae_lambda"] * next_nonterminal * last
                advantages[t] = last
            returns = advantages + values
            obs_batch=torch.cat(obs_buf); act_batch=torch.cat(action_buf); old_logp=torch.cat(logp_buf)
            old_values=values.reshape(-1); adv=advantages.reshape(-1); ret=returns.reshape(-1)
            adv=(adv-adv.mean())/(adv.std()+1e-8)
            order=torch.randperm(len(ret),device=device); actor_losses=[]; critic_losses=[]; entropies=[]; kls=[]; clip_fracs=[]
            network.train()
            for _ in range(ppo_cfg["update_epochs"]):
                for idx in order.split(ppo_cfg["minibatch_size"]):
                    new_logp, entropy, new_value = network.evaluate(obs_batch[idx], act_batch[idx])
                    ratio=(new_logp-old_logp[idx]).exp()
                    surr1=ratio*adv[idx]; surr2=ratio.clamp(1-ppo_cfg["clip_ratio"],1+ppo_cfg["clip_ratio"])*adv[idx]
                    actor_loss=-torch.minimum(surr1,surr2).mean()
                    critic_loss=0.5*(new_value-ret[idx]).square().mean()
                    loss=actor_loss+ppo_cfg["value_coefficient"]*critic_loss-ppo_cfg["entropy_coefficient"]*entropy.mean()
                    optimizer.zero_grad(set_to_none=True); loss.backward(); torch.nn.utils.clip_grad_norm_(network.parameters(),ppo_cfg["max_grad_norm"]); optimizer.step()
                    actor_losses.append(float(actor_loss.detach().cpu())); critic_losses.append(float(critic_loss.detach().cpu())); entropies.append(float(entropy.mean().detach().cpu())); kls.append(float((old_logp[idx]-new_logp).mean().detach().cpu())); clip_fracs.append(float(((ratio-1).abs()>ppo_cfg["clip_ratio"]).float().mean().detach().cpu()))
            row={"update":update+1,"interactions":interactions,"wall_time_s":base_wall_time+time.monotonic()-start_time,"rollout_s":time.monotonic()-t0,"mean_reward":float(rewards.mean().cpu()),"mean_return":float(returns.mean().cpu()),"actor_loss":float(np.mean(actor_losses)),"critic_loss":float(np.mean(critic_losses)),"entropy":float(np.mean(entropies)),"approx_kl":float(np.mean(kls)),"clip_fraction":float(np.mean(clip_fracs)),"mean_episode_reward":float(rewards.sum(0).mean().cpu()),"parameter_class":"SIM_ONLY"}
            if (update+1)%args.validation_every==0 or update==total_updates-1:
                network.eval(); row["validation"]=validation(env,network,torch,np,args.seed+500000+(update+1)*1009,device,args.validation_episodes); obs,_=env.reset(seed=args.seed+update+1); observation=obs["policy"].clone()
            metrics.append(row); metric_log.write(json.dumps(finite(row))+"\n"); metric_log.flush(); print("PPO_UPDATE="+json.dumps(finite(row)),flush=True)
            state={"model":network.state_dict(),"optimizer":optimizer.state_dict(),"update":update+1,"interactions":interactions,"config":ppo_cfg,"environment_config":task,"seed":args.seed,"elapsed_wall_time_s":base_wall_time+time.monotonic()-start_time}
            torch.save(state,args.output/"checkpoint_latest.pt")
            if row.get("validation") is not None: torch.save(state,args.output/f"checkpoint_update_{update+1:05d}.pt")
    actor=save_actor(torch,network,args.output/"actor.pt",ppo_cfg)
    with torch.inference_mode():
        sample=torch.zeros(1,31); t0=time.perf_counter();
        for _ in range(1000): actor(sample)
        actor_forward_ms=(time.perf_counter()-t0)*1000/1000
    summary={"status":"SIM_ONLY_RUN_COMPLETE","parameter_class":"SIM_ONLY","algorithm":"standard_ppo","seed":args.seed,"updates":total_updates,"environment_interactions":interactions,"wall_time_s":base_wall_time+time.monotonic()-start_time,"average_env_step_s":(base_wall_time+time.monotonic()-start_time)/(interactions/max(env.num_envs,1)),"ppo_device":str(device),"actor_forward_ms":actor_forward_ms,"config_sha256":digest(args.ppo_config),"environment_config_sha256":digest(args.env_config),"source_sha256":{"train_ppo.py":digest(Path(__file__))},"metrics":metrics,"checkpoint":str(args.output/"checkpoint_latest.pt"),"actor":str(args.output/"actor.pt"),"hardware_connected":False}
    (args.output/"summary.json").write_text(json.dumps(finite(summary),indent=2)+"\n")
    env.close()
    app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
