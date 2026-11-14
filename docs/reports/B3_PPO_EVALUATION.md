# B3：PPO 独立仿真评估（SIM_ONLY）

> 状态：`PARTIAL（SIM_ONLY）`。评估脚本实现了冻结成功判据和统一动作链路；PPO、zero、random、IK+PD 已各完成 100 回合同初始状态评估，IK+PD+gravity 因第二环境初始化无输出而未完成。

## 评估合同

- 评估入口：[`scripts/evaluate_ppo.py`](../../scripts/evaluate_ppo.py)。它在不更新网络权重的独立进程中读取 TorchScript Actor，并运行同一 Isaac Lab 环境。
- 策略集合：PPO、zero、random、IK+PD、IK+PD+gravity；每个策略使用同一组保留初始状态种子，失败 episode 计入成功率分母。
- 轨迹和终止：半径 10 mm、周期 8 s、episode 10.5 s；成功必须覆盖完整圆周，恒速段 RMSE ≤5 mm、P95 ≤5 mm、最大误差 ≤8 mm，且无异常 `terminated` 并以正常 `truncated` 结束。阈值来自 B2-S1，评估前已冻结。
- 记录：逐步参考位置、实际 FK 位置、q/dq/ddq、动作、目标、力矩、奖励分量、碰撞/限位/非有限终止原因；每个已完成策略输出 `summary.json`、XY 和误差图，完整 transitions 仅保留本地 `runs/`。原脚本的跨策略 `comparison.json` 因 gravity 初始化阻塞未生成，脱敏聚合另存于 `data/B3_FINAL_EVAL_SUMMARY.json`。

## 运行命令

```bash
FACTORY_ROOT=/home/ubuntu2004/Documents/Codex/2026-10-23/the-github-repository-has-now-been
PYTHONDONTWRITEBYTECODE=1 bash "$FACTORY_ROOT/outputs/isaac-factory-repro/scripts/run_sim.sh" \\
  scripts/evaluate_ppo.py --urdf <private-reviewed-urdf> --usd <private-reviewed-usd> \\
  --actor runs/<final-run>/actor.pt --output runs/b3_eval_final \\
  --policies ppo zero random ik ik_gravity --episodes 100 \\
  --num_envs 4 --headless --device cpu
```

实际使用的私有路径和命令 JSON 应保存于忽略目录 `runs/b3_eval_final/command.json`，不得提交厂商资产。

## 当前证据

- `runs/b3_smoke/summary.json` 的两次 validation 均成功率 `0/1`，RMSE `14.326 mm` 和 `16.695 mm`；这是训练过程预检，不是独立测试。
- 评估脚本已经修正为 `torch.no_grad()`，避免 PhysX-backed tensor 在 reset/step 生命周期中被 `inference_mode` 标成不可写；修正后的 PPO 以及三组对照各 100 episode 结果已保存。

截至本报告更新，PPO、zero、random、IK+PD 四组均完成 100 回合，使用同一组保留初始状态；公共聚合结果见 [`data/B3_FINAL_EVAL_SUMMARY.json`](data/B3_FINAL_EVAL_SUMMARY.json)。

| 策略 | 成功率 | 完整圆周率 | 异常终止率 | 恒速圆 RMSE | 恒速圆 P95 | 恒速圆最大误差 | 主要终止 |
|---|---:|---:|---:|---:|---:|---:|---|
| PPO | 0/100 | 0/100 | 100/100 | 3.535 mm | 5.866 mm | 5.890 mm | orientation |
| zero | 0/100 | 100/100 | 0/100 | 15.715 mm | 21.885 mm | 21.954 mm | 无 |
| random | 0/100 | 0/100 | 100/100 | 12.076 mm | 15.618 mm | 16.296 mm | orientation 99、tracking_workspace 1 |
| IK+PD | 100/100 | 100/100 | 0/100 | 3.406 mm | 3.708 mm | 3.712 mm | 无 |

PPO 的独立结果为平均全段 RMSE `3.345710 mm`、平均持续时间 `8.503 s`，100 回合全部因 `orientation` 异常终止；因此即使 RMSE 接近阈值，也没有完成冻结成功判据。zero 走满时域但误差过大，random 不能存活，IK+PD 作为已知传统基线在所测局部任务上通过。IK+PD+gravity 的第二环境初始化超过 15 分钟无输出，已终止该无输出进程；本批次不对它推断结果，既有 B1-S1 的单轨迹 gravity 对照仍单独有效。

## 未完成与判定

本批次没有生成原脚本的 `comparison.json`：gravity 环境初始化无输出导致进程被终止；四组已完成策略的脱敏 aggregate 已单独保存。不要把缺失的 gravity 结果当作成功或失败，也不得将仿真成功推出 Sim2Real 成功，B0 实机门禁继续 `BLOCKED`。

## 学习验收问题

1. 评估脚本如何保证失败 episode 留在成功率分母中？
2. 为什么 PPO、IK+PD 与 IK+PD+gravity 的控制器差异必须单独记录？
3. `evaluate_circle_success` 的完整圆周、RMSE、P95、最大误差和终止条件如何共同决定 success？
