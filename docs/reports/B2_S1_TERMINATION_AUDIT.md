# B2-S1：姿态终止、动作接口与成功判据审查

日期：2026-11-14（KST）
输入基线：`700187dc76fee20d6905f2fab5892d628feea27a`
范围：**SIM_ONLY**；没有 PPO 训练、真实机器人连接或电机使能。

## 结论

本审查保持 `orientation_error_stop_rad = 0.05 rad`，没有为了提高随机动作存活率而放宽它。该值来自 B2-S0 的固定姿态实验配置（[配置](../../configs/b2_s0_circle_sim_only.json#L25-L29)），不是厂商参数、实机精度或安全边界。固定姿态是任务定义的一部分；碰撞、关节限位、奇异性、非有限状态、速度和加速度仍由独立终止逻辑检查。

当前环境的终止返回语义正确：安全运行到 630 个控制步返回 `terminated=False, truncated=True`；姿态、碰撞、越限、非法动作和非有限反馈返回 `terminated=True, truncated=False`。代码见 [`_get_dones`](../../src/robot_rl_sim2real/isaac_circle_env.py#L262-L288)。

冻结的独立成功判据已加入 [`evaluate_circle_success`](../../src/robot_rl_sim2real/circle_rl.py#L103-L145)：恒速段参考相位覆盖至少一整圈，且恒速段末端位置 RMSE ≤ 5 mm、P95 ≤ 5 mm、最大误差 ≤ 8 mm；整个 episode 不得 `terminated`，正常到达时间上限时必须是 `truncated`。`truncated=True` 本身不能构成成功。阈值是本阶段的 SIM_ONLY 验收指标，训练前冻结，不根据训练结果回调。

## 姿态阈值检查

终止姿态误差是 `link6` 当前 FK 姿态相对于 `q_home` FK 姿态的旋转角（[`assess`](../../src/robot_rl_sim2real/isaac_circle_env.py#L192-L225)）。当前 reward 只使用位置误差、动作幅值和动作变化；姿态不是 reward 项。31 维 observation 含实测 `q`、`dq`、位置误差、参考速度、上一动作、上一目标与实测 q 的差、episode 时间，但没有单独的姿态误差标量（[observation contract](B2_S0_OBSERVATION_ACTION.md)）。因此策略可以从关节状态间接推断姿态，尚未得到直接的姿态学习信号。

### 已有随机动作证据

证据来自 B2-S0 普通 PD、2 个 CPU 环境、固定种子、630 个批量控制步的脱敏汇总（[测试数据](data/B2_S0_TEST_RESULTS.json)及本地 `runs/b2_s0/full_pd_005/transitions.jsonl`）：

| 项目 | 结果 |
|---|---:|
| 随机动作观测到的姿态终止 | 14 次 |
| 两个首个 episode 终止步 | 56、102 |
| 对应时间 | 0.9333 s、1.7000 s |
| 终止姿态误差 | 0.05068、0.05265 rad |
| 对应位置误差 | 10.16、16.60 mm |
| 这些行的终止原因 | 仅 `orientation` |
| 位置、速度、碰撞、关节限位 | 未触发 |

终止前随机样本的姿态误差 P50/P90/P95/P99/最大值约为 `0.0265/0.0450/0.0473/0.0502/0.0526 rad`；位置误差对应分位数约为 `7.80/14.27/16.24/26.35/29.02 mm`。这说明随机动作在姿态偏离前通常已经产生较大的位置跟踪误差；姿态终止不是唯一的学习障碍，但会把部分坏轨迹很早截断。

传统 IK 诊断输入在相同终止逻辑下完整运行 630 步，恒速圆段 RMSE 约 `3.406 mm`，全序列最大姿态误差约 `0.01473 rad`，没有异常终止。零动作也完成 630 步并以 `truncated=True` 结束。该结果支持保留 0.05 rad 作为当前固定姿态任务的实验约束。

### 有限候选比较

以下是对已记录随机轨迹的**轨迹级敏感性检查**，不是把阈值改掉后重新运行 PhysX 的反事实实验：

| SIM_ONLY 阈值 | 已记录随机行超过阈值 | 解读 |
|---:|---:|---|
| 0.05 rad | 14 | 当前配置；均在越过门槛后终止 |
| 0.075 rad | 0 | 记录在 0.05 rad 处已重置，不能证明放宽后能安全存活 |
| 0.10 rad | 0 | 同上；不能作为放宽安全边界的证据 |
| 0.20 rad | 0 | 同上；更不代表固定姿态任务仍满足要求 |

因此本阶段**不修改阈值**。若后续策略在位置指标合格、但姿态误差长期接近该门槛，需要先在独立 SIM_ONLY 消融中同时增加姿态观测和姿态误差项，再决定任务是否仍为固定姿态；不能单独放宽终止门槛。实机姿态容差仍 UNKNOWN。

选择 0.05 rad 的唯一当前数值依据是实验内部的相对量级：传统 IK 诊断全序列最大姿态误差约 0.01473 rad，门槛约为其 3.4 倍，且没有截断该基线；这不是统计置信区间，也不是硬件认证。

## Action 与传统 IK 对照

动作契约是相对于上一位置目标的 6 维增量。`map_action` 先把网络输出限制到 `[-1,1]`，按每步 `0.25` 的归一化变化率限制，再计算

```text
b_t       = a_prev + clip(u_t - a_prev, -0.25, 0.25)
q_target  = clip(q_target_prev + 0.003 rad * b_t, q_local_lower, q_local_upper)
a_applied = (q_target - q_target_prev) / 0.003 rad
```

实现见 [`map_action`](../../src/robot_rl_sim2real/circle_rl.py#L36-L54)，环境在 [`_pre_physics_step`](../../src/robot_rl_sim2real/isaac_circle_env.py#L232-L251) 中唯一调用该映射。目标相对上一目标累积，并始终裁剪到本地区间，不会无限积累。重置时目标、上一动作、reward history、上一速度和时钟全部清零或恢复到 checked q（[`_reset_idx`](../../src/robot_rl_sim2real/isaac_circle_env.py#L326-L347)）。

传统 IK 诊断没有绕过 Action 接口：测试脚本把离散 IK 目标与 `env.target` 的差换算为同一个 6 维 action，然后调用 `env.step`（[`test_circle_rl_env.py`](../../scripts/test_circle_rl_env.py#L141-L155)）。对 3,760 个已记录过渡独立重算，目标映射最大差为 `0`；所有轨迹 `target_clipped_count=0`。60 Hz 下单步目标增量上限对应名义 `0.003*60 = 0.18 rad/s`。B1-S1 已选圆轨迹恒速段的参考关节速度最大约 `0.10423 rad/s`，因此该动作上限足够覆盖当前圆轨迹；PD 实际速度的轻微超调仍由仿真控制器和速度终止门处理。

## Reset、终止与训练接口门

- `is_finite_horizon=True`，episode 时长 `10.5 s = 630` 个 60 Hz 控制步。
- 故障优先于 timeout：`failed` 为 `terminated`，只有没有故障且达到步数上限才置 `truncated`。
- reset 后实际 q 在 `q_home ±0.001 rad` 内，上一目标等于该 q；上一动作、reward history、速度 history、非法动作历史、碰撞标志和 episode 时钟复位。
- 已有双环境独立 reset、固定种子重复性、碰撞正例、关节限位、NaN action 和 NaN feedback 注入均通过；环境间只过滤碰撞，机器人内部 self-collision 保留。
- 当前还没有 PPO runner 或 value bootstrap 封装。后续 PPO 必须读取 `truncated` 与 `is_finite_horizon` 语义，不能把异常 `terminated` 当作正常时间截断，也不能把“走满 630 步”单独当作成功。

## 冻结评价实现与结果

`evaluate_circle_success` 是纯 CPU、无 Isaac 依赖的评价函数，输入一条 episode 的 transition records，输出圆周覆盖、RMSE、P95、最大误差、terminated/truncated 和 success。用 B2-S0 传统 IK 诊断 episode 回放得到：

```text
complete_circle       = true
phase_span            = 6.283185307179586 rad
circle_rmse           = 0.0034063584 m
circle_p95            = 0.0037083564 m
circle_max            = 0.0037121223 m
terminated             = false
truncated              = true
success                = true
```

对应的零动作和随机动作不能通过该判据：零动作没有终止但位置误差超过指标；随机动作没有完整圆周且 `terminated=True`。纯 CPU 单测新增完整圆周、错误阈值、提前 terminated 和缺少 timeout 场景，均通过。

## 状态

**PARTIAL（SIM_ONLY，具备进入 B3 前置 smoke 的证据，但不等同于 PPO 已可训练或实机可部署）。** 姿态门、动作链路、reset 与 terminated/truncated 已审查；固定姿态是否应作为策略学习目标仍需训练前消融。B0 实机接口保持 BLOCKED，真实 SDK 的目标覆盖、反馈时序、姿态/TCP 标定与安全停止尚未验证。
