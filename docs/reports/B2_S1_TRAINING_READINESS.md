# B2-S1：PPO 前环境训练准备审查

日期：2026-11-14（KST）；输入基线：`700187dc76fee20d6905f2fab5892d628feea27a`。范围为 **SIM_ONLY**。本阶段没有启动 PPO、没有连接真实机器人，也没有修改 Isaac Sim、Isaac Lab、CUDA 或驱动。

## 结论

结论为 **PARTIAL**：B2-S0 环境的奖励、终止语义、动作接口和 CPU 回归已经完成，B2-S1 的 `exp_linear` 候选已经在完整 PD 与 `pd_gravity` 2 环境 episode 中通过。环境还没有 PPO runner、GPU 吞吐量证据或训练曲线，因此不能把本报告称为 PPO 已可收敛，也不自动进入 B3。

## 候选 reward 的完整回归

B2-S1 使用 [`configs/b2_s1_reward_sim_only.json`](../../configs/b2_s1_reward_sim_only.json)，其位置项为：

```text
r_pos = (0.03 + exp(-position_error / 0.005)) / 630
```

动作幅值和动作变化项沿用 B2-S0，故每步总奖励下界仍为 `7.9365e-6`。完整运行使用 2 个 CPU 环境、Isaac Sim 4.5.0、Isaac Lab 2.1.1、10.5 s/630 控制步；原始日志和逐步记录位于 git 忽略的 `runs/`。

| 控制器 | 诊断 IK 恒速圆 RMSE | 诊断回报（env0/env1） | 零动作 RMSE | 随机动作首个终止步 | 结果 |
|---|---:|---:|---:|---:|---|
| 普通 `pd` | 3.406 mm | 0.53058 / 0.53068 | 14.289 / 14.015 mm | 56 / 102（姿态） | PASS |
| `pd_gravity`（SIM_ONLY） | 0.0765 mm | 1.01660 / 1.01645 | 12.787 / 12.623 mm | 72 / 204（姿态） | PASS |

两种控制器都完成了 startup、circle、stop、hold 四个阶段；位置 FK 最大误差分别为 `3.20e-7 m` 和 `2.98e-7 m`，没有碰撞、越限或 NaN。`pd_gravity` 是仿真模型重力前馈对照，不是已确认的真实 SDK 能力。两次运行的输入配置和 SHA-256 写在各自的 `runtime_metadata.json` 中，控制器结果没有混合。

冻结的纯 CPU 成功判据为恒速段相位覆盖至少 `2π`、RMSE ≤5 mm、P95 ≤5 mm、最大误差 ≤8 mm、无 `terminated`，并以正常 `truncated` 结束。普通 PD 诊断回放为 `3.406/3.708/3.712 mm`，重力前馈为 `0.076/0.077/0.077 mm`（RMSE/P95/最大），两者均通过；走满 630 步本身不构成成功。

## 吞吐量预检

测量命令使用 [`scripts/benchmark_circle_rl_env.py`](../../scripts/benchmark_circle_rl_env.py)，每次启动后先 warm-up 4 步，再计时 64 个控制步；不含 Isaac 启动时间。结果来自 B2-S1 的 CPU 场景，输出保留在 `runs/b2_s1_bench_n{1,2,4}/result.json`。

| 环境数 | 控制步/s | env-step/s | 物理残差/步循环 | FK 计时 | 碰撞计时 | observation | reward |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 38.45 | 38.45 | 0.206 s/64步 | 0.433 s | 0.879 s | 0.037 s | 0.018 s |
| 2 | 18.93 | 37.87 | 0.225 s/64步 | 1.000 s | 1.891 s | 0.076 s | 0.025 s |
| 4 | 10.72 | 42.89 | 0.235 s/64步 | 1.762 s | 3.547 s | 0.115 s | 0.030 s |

这里的 `env-step/s` 是并行环境交互数；`control step/s` 是所有环境共享的控制时钟。64 步计时中，`assess` 的 FK 和碰撞检查分别以方法包装计时，外层 `pre_physics`/`dones` 已包含这些嵌套调用，报告中没有重复相加。剩余项包括 PhysX stepping、同步、状态读取和框架开销。4 个环境的聚合吞吐量略高，但每个环境的控制时钟只有约 10.7 Hz，说明 CPU 方案适合小规模前置检查，不适合据此宣称实时或高吞吐训练。

本机只读资源检测为 16 个逻辑 CPU、Intel i7-10700F、约 30 GiB 可用内存、RTX 3070 8 GiB（检测时显存使用约 1.73 GiB）。本阶段没有启动 GPU 训练，也没有测量 GPU Isaac Lab 吞吐量。

## PPO 接口门

- `terminated` 只表示姿态、碰撞、越限、非有限状态、速度/加速度等异常；正常到达 630 步且没有故障返回 `truncated=True`。
- `is_finite_horizon=True`，因此后续 PPO 封装必须区分 time-limit bootstrap 和真正失败，不能把两类 done 合并。
- reset 会清空上一动作、上一位置目标差、上一速度、非法动作/碰撞标志和 episode 时钟；双环境选择性 reset、固定种子重放和故障注入回归通过。
- 传统 IK 目标先换算为和 Actor 完全相同的 `map_action` 输入，再调用 `env.step`；没有绕过 action 接口。
- 当前仍无 PPO runner、值函数 bootstrap 实现、GPU 训练配置或收敛验收曲线。下一阶段如获单独授权，只应先运行短 smoke，冻结本报告的成功判据，并继续保留碰撞、限位和非有限检查。

## 回归与限制

候选配置下的零动作、随机动作、诊断 IK、双环境 reset、碰撞正例、关节越限、NaN action/feedback、固定种子重复性均通过；完整普通 PD 与 `pd_gravity` 对照也通过。B2-S0 的旧 clipped quadratic 仍保留在原配置和历史证据中，未被覆盖。

尚未验证的内容包括 GPU 吞吐量、PPO 优化器/runner 的 bootstrap 细节、更多初始状态和工作空间、真实 SDK 的反馈延迟与连续指令语义，以及任何真实机器人安全属性。所有控制器增益、动作尺度、速度/加速度门和 reward 参数继续标记为 SIM_ONLY。
