# B3：训练性能与资源记录（SIM_ONLY）

> 草稿状态：`PARTIAL`。本文记录已测 smoke/正式片段和已知瓶颈；不将网络 forward 时间称为完整 observation-to-command 延迟。

## 环境与安全边界

- Ubuntu 22.04.5；RTX 3070 8 GB，驱动 `580.178.04`；Isaac Sim 4.5.0、Isaac Lab 2.1.1、PyTorch 2.7.0+cu128。
- 复用 Factory 隔离环境；未升级驱动、CUDA、Isaac Sim、Isaac Lab、ROS 2 或 PyTorch。
- Isaac 物理运行 `--device cpu`，PPO learner 可运行 `cuda:0`。self-collision、关节限位、非有限检查、FK 和 collision geometry 检查保持开启。

## 已测吞吐

B2-S1 CPU 基准（同一环境实现）为 1/2/4 环境约 `38.45/37.87/42.89 env-step/s`。B3 smoke 的 2 环境 runner 在 512 interactions、约 61.66 s 墙钟完成，包含 validation、PPO 更新和 checkpoint 写盘；该数字不能直接等同于稳态 physics FPS。正式 4 环境训练共 399,360 interactions、metrics 墙钟 9,447.55 s，约 `42.3 env interactions/s`（summary 含收尾时间约 `42.1/s`）。该吞吐包含 rollout、PPO 更新和 checkpoint，不能当作 physics-only FPS。

已知主要成本是 CPU FK 和碰撞几何筛查；当前未关闭安全检查。最终 Actor 的独立 `scripts/infer_actor.py` CPU forward 测量为约 `0.0346 ms`（1000 次平均），训练 runner 的摘要测量为 `0.0506 ms`；两者测量上下文不同，均只表示网络 forward，不含 FK、观察构造、Isaac step、动作安全层或真实 SDK 通信。

## 资源预算与异常

本轮训练预算为墙钟不超过 5 小时；单个 seed 在约 157.5 min 内完成 780 updates。训练未显示 NaN/Inf，PPO loss、KL、entropy 有限。未对峰值显存作持续采样，也未完成多个 seed；报告只声明这一 seed 的证据，不将其泛化为算法稳定性。

## 优化决策门

先保留可审计的 CPU FK、碰撞和故障检查；只有在优化前后通过相同 FK、碰撞正负例、reset 和 action 回归后，才可接受批量化或减少同步。任何吞吐优化都不能删除安全约束。若最终吞吐不足，保持 `B3_PARTIAL` 并把瓶颈记录为进入后续阶段的风险。

## 学习验收问题

1. 为什么 smoke 的平均 wall time 不能直接作为 physics-only FPS？
2. 当前吞吐瓶颈为何集中在 CPU FK 与碰撞筛查，而不是 Actor forward？
3. 优化碰撞或 FK 计算时，哪些回归证据必须保持不变？
