# B3：PPO 训练记录（SIM_ONLY）

> 状态：`PARTIAL（SIM_ONLY）`。本报告记录已落盘的 PPO smoke、正式训练和冻结 validation；任务成功与否以独立评估和 `docs/STATUS.md` 为准。没有连接真实机器人，也没有发送 ROS/CAN 指令。

## 范围与输入

- 输入基线：`823b8fd3ebba0c0416ed55f3abdfc8a96e60d696`。
- 环境：Ubuntu 22.04.5、Isaac Sim 4.5.0、Isaac Lab 2.1.1（commit `90b79bb2d44feb8d833f260f2bf37da3487180ba`）、Python 3.10.12、PyTorch 2.7.0+cu128、RL-Games 1.6.1；Isaac 仿真使用 Factory 隔离环境并强制 `--device cpu`，PPO learner 可使用 `cuda:0`。
- 输入模型：B0-R1A 审查的私有 URDF/USD。URDF SHA-256 为 `78c1b35c65fe747876c19c06c3ca3492d22aeac4416c536f585da494ef6842ae`；私有 USD 位于 `model_assets/local/`，不提交仓库。
- 任务：B2-S1 圆形轨迹环境，31 维实际状态观测、6 维有界关节位置增量、SIM_ONLY 普通 PD、`exp_linear` 位置奖励；半径 10 mm、周期 8 s、episode 10.5 s。
- PPO：自包含标准 PPO；Actor/Critic 各两层 128 单元 MLP，tanh-normal 动作并包含 tanh Jacobian 的 log-probability，GAE，PPO clipping，有限时域配置 `value_bootstrap=false`。完整超参数见 [`configs/ppo/circle_ppo_sim_only.json`](../../configs/ppo/circle_ppo_sim_only.json)。

`value_bootstrap=false` 是本阶段固定合同：runner 将 `terminated` 和 `truncated` 合并为 rollout 边界，GAE 不跨边界 bootstrap。配置项不是一个已验证的可切换实验开关；若要改为只对异常终止截断，必须同时改代码和测试。

## 可复现实验命令

Isaac Sim 由既有 Factory launcher 启动（路径仅适用于本机）：

```bash
FACTORY_ROOT=/home/ubuntu2004/Documents/Codex/2026-10-23/the-github-repository-has-now-been
PYTHONDONTWRITEBYTECODE=1 bash "$FACTORY_ROOT/outputs/isaac-factory-repro/scripts/run_sim.sh" \\
  scripts/train_ppo.py --output runs/b3_seed20261201 \\
  --seed 20261201 --total-steps 99840 --rollout-steps 128 \\
  --num-envs 4 --validation-every 20 --validation-episodes 4 \\
  --headless --device cpu --ppo-device cuda:0
```

首次进程在 update 7 后停止；随后使用同一 Factory launcher、同一输出目录和
`--resume runs/b3_seed20261201/checkpoint_latest.pt` 续训至 update 780。恢复时环境状态按固定 seed reset，checkpoint 只恢复 Actor/Critic 与 optimizer，不恢复 PhysX 状态。

输出目录和本机完整命令日志在 `runs/`（被 `.gitignore` 忽略）；公开仓库只保留脚本和配置。

## 已落盘结果

### Smoke

`runs/b3_smoke/summary.json` 记录了 seed `20261201`、2 个环境、2 次 update、512 次环境交互、约 61.66 s 墙钟；该目录包含可加载的 `checkpoint_latest.pt`、`actor.pt` 和两次 validation。PPO loss、KL、entropy、checkpoint 保存和 TorchScript 导出均有限且成功。两次 validation 成功率均为 0：第 1 次完整一圈但 RMSE `14.326 mm`，第 2 次在相位约 `5.576 rad` 处异常终止，RMSE `16.695 mm`。这只是 runner smoke 证据，不能称为策略收敛。

### 正式训练结果

`runs/b3_seed20261201/metrics.jsonl` 已完成 780 次 update（4 环境、每次 512 交互；累计 `399,360` 次环境交互）。metrics 记录的总墙钟时间为 `9,447.55 s`（约 157.5 min；summary 包含收尾时间为 `9,489.61 s`），在 5 小时预算内。update 1–7 来自早期片段，使用修正前的 GAE done 索引；update 8–780 使用 `1-dones[t]` 的修正实现，报告保留这一分界。

最终 update 780 的 4 episode validation 成功率为 `0/4`。四个 episode 均以异常 `terminated` 结束，圆周相位覆盖 `5.864–5.904 rad`，尚未达到完整 `2π`；恒速段 RMSE 为 `3.525–3.544 mm`（均值 `3.536 mm`），P95 为 `5.876–5.888 mm`，最大误差为 `5.884–5.912 mm`。因此 RMSE 和最大误差接近目标，但完整圆周、P95 和正常 `truncated` 条件未同时满足冻结成功判据。训练过程的 PPO loss、KL、entropy 均保持有限，checkpoint 和 TorchScript Actor 均已保存。

训练脚本的 `summary.json` 使用 `status: SIM_ONLY_RUN_COMPLETE` 表示 runner 完成（旧运行摘要中可能保留 `PASS` 字段）；这两者都不能代表任务成功。阶段状态以冻结 validation/独立评估结果为准，当前为 `B3_PARTIAL`。

## 失败、限制与未知

- smoke 与正式 validation 均未通过冻结圆轨迹成功判据，因此不能标记 `B3_PASS`。
- `--resume` 已支持同一输出目录追加 metrics；checkpoint 不保存 PhysX 环境状态，续训从明确 seed reset，且 update 1–7 与后续修正 GAE 的边界保留在审计记录中。
- 最终 Actor 的 100 episode PPO 独立测试已落盘；同一初始状态集合下的 zero、random、IK+PD 也已完成，IK+PD+重力前馈的新环境初始化无输出而被终止，未从缺失结果推断结论。
- 所有增益、动作尺度、速度/加速度/力矩限制和奖励参数均为 `SIM_ONLY`；实机 SDK 的目标覆盖、反馈频率和停止语义仍是 B0 `BLOCKED`。

## 决策门

只有在正式训练达到预算内的目标步数、checkpoint 可恢复、最终 Actor 独立加载成功，并在冻结成功判据下完成独立测试后，才可把 B3 状态更新为 `B3_PASS`；否则保持 `B3_PARTIAL`，保留失败轨迹和日志，并先修正环境/接口问题。该报告不授权 B4、B5 或实机控制。

## 学习验收问题

1. `train_ppo.py` 为什么对 tanh-normal 动作在 log-probability 中加入 Jacobian 修正？若直接裁剪高斯样本会破坏什么一致性？
2. 当前有限时域配置下，`terminated` 和 `truncated` 如何参与 GAE；为什么不能把两者简单视为同一种 done？
3. `runs/b3_smoke/summary.json` 中 validation 失败说明了什么，为什么不能由一次 smoke 的 reward 判断训练已成功？
