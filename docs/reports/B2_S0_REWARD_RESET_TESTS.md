# B2-S0：奖励、Reset 与无训练测试

日期：2026-11-13（KST）；输入基线：`1704d1b8b3b68f20128061962c96212e466f929c`。**PASS（限定 SIM_ONLY 环境和所测两环境场景）。** 没有训练策略，也没有连接实机。各项测试的机器可读结果见[脱敏测试汇总](data/B2_S0_TEST_RESULTS.json)。

## 奖励：恰好三个分量

令 `N=630`，`e=||p_d−p_ee||₂`（m），`a` 是经过动作变化率和位置裁剪后实际应用的6维归一化动作，`a_prev` 是上一步的实际动作。每步分别记录：

```text
E       = min((e / 0.005 m)^2, 1)
r_pos   = (1.03 − E) / N
r_act   = −0.005 × mean(a²) / N
r_delta = −0.005 × mean((a − a_prev)²) / N
r_total = r_pos + r_act + r_delta
```

误差按 5 mm 缩放并截断；无姿态或碰撞奖励项。姿态、碰撞及非法状态由[独立终止逻辑](../../src/robot_rl_sim2real/isaac_circle_env.py#L183)处理。为了使提前结束不会通过跳过未来负奖励获益，跟踪项含 `0.03/N` 的正下限；因 `|a_i|≤1`、`|a_i−a_prev_i|≤2`，单步总奖励至少 `(0.03−0.005−0.020)/630=7.94e-6`。在同一 episode 中，继续执行总能累积正回报；一步就结束的理想上限约 `0.001635`，完美跟踪且零动作的完整 episode 理论上限为 `1.03`。这要求后续训练封装正确处理终止和有限时域，不是对所有训练配置防作弊的证明。

5 mm 以上的截断使跟踪项趋于常数，可能产生稀疏学习信号；因此还需在 B2-S1 检查可学习性，不能由此处的公式测试推断 PPO 会收敛。实际零动作、随机动作与传统 IK 驱动的返回分别比较，不能仅比较经过不同长度 episode 的单步均值。

## 运行与测试

Isaac Sim 4.5.0、Isaac Lab v2.1.1 commit `90b79bb2d44feb8d833f260f2bf37da3487180ba`、Python 3.10.12、PyTorch 2.7.0+cu128、NumPy 1.26.4；Factory `factory-g01` 隔离 venv，PhysX CPU，两个独立环境，seed `20261113`。这不验证 GPU 并行训练。运行前核对私有 URDF 和四个 USD 层与 B0-R1A 的 SHA-256；不修改或提交原始或派生模型。

| 项目 | 结果与范围 |
|---|---|
| Observation | `2×31`、有限；逐步从 q/dq、CPU FK、参考及已知动作/目标重建。排除 reset 的连续步比较通过。 |
| Action 边界 | 输入全1的第一个目标增量为每轴 `0.00075 rad`；CPU 测试覆盖裁剪、变化率、越界与非有限输入。 |
| Reward | 三项逐步相加且有限；已检查正常与异常终止处理以及零动作/随机/诊断驱动的回报。 |
| Reset | 前50个样本均通过局部限位、朝向、雅可比、联合凸包检查并走过物理步；最小奇异值约 `0.05424`，初始最大位置偏差约 `0.573 mm`。所有后续 reset 仍执行相同检查。 |
| Episode 与参考 | 诊断 IK 输入完成630步、10.5 s；加速、一圈、减速、保持均覆盖，独立 FK 与 Isaac 位置最大差约 `3.20e-7 m`。 |
| 碰撞/越限 | 有意折叠正例同时触发几何与 PhysX 接触，注入模型关节越限被终止；另一环境未受影响。 |
| 非有限输入 | 注入 NaN 动作和状态读取故障均独立终止并 reset；没有把 NaN 写入 PhysX。 |
| 零动作 | 两个 episode 均达到630步时间上限，末端全段 RMSE 约 `14.29/14.02 mm`；这只是保持目标，不能完成圆跟踪。 |
| 随机动作 | 两个首个 episode 分别在56/102步因姿态偏差终止；在630个批量控制步中共14次终止。它是故障/环境测试，不是训练效果。 |
| 多环境与固定种子 | 只 reset 或碰撞注入 env0 时，env1 状态、目标、动作、时钟和 RNG 未发生串扰；相同 seed/动作重放32步最大差0，不同 seed 改变 reset。 |

作为环境可达性诊断，传统 B1-S1 IK 参考驱动的两个环境完整走完；恒速圆位置 RMSE 约 `3.406 mm`，累计回报约 `0.5469/0.5468`。普通 PD 零动作回报约 `0.0836/0.1051`；随机动作首个提前终止 episode 回报约 `0.00737/0.01119`。这只是本次两环境、一个种子和指定动作序列的比较，不是策略性能或训练成功率。

额外离线重算了3,760个连续动作目标，最大差为0；独立重算奖励与记录的最大差约 `4.34e-19`；参考位置与 B1-S1 轨迹按时间插值的最大差约 `5.55e-17 m`。上述三类正常运行的最大关节速度和命令力矩分别为 `0.2372 rad/s`、`2.183 Nm`，没有目标位置裁剪。证据和具体计算范围见[汇总](data/B2_S0_TEST_RESULTS.json)。

独立 `pd_gravity` 模式只做了两环境32步（0.533 s）启动短测，通过维度、动作、reset、FK和有限值检查；并未运行完整重力对照 episode。B1-S1 的完整独立 PD/重力对照结果仍在[既有基线报告](B1_S1_CLASSICAL_BASELINE.md)。

## 复现命令与原始证据

```bash
B2_FACTORY=/home/ubuntu2004/Documents/Codex/2026-10-23/the-github-repository-has-now-been
B2_URDF=/tmp/student_ros_b0_audit.sYOXbS/student_ros/src/episode1_urdf_1113/urdf/episode1_urdf_1113.urdf
B2_LAUNCHER="$B2_FACTORY/outputs/isaac-factory-repro/scripts/run_sim.sh"
python3 -m unittest discover -s tests -q
env FACTORY_VENV="$B2_FACTORY/work/envs/factory-g01" PYTHONDONTWRITEBYTECODE=1 timeout --signal=TERM --kill-after=10s 600s "$B2_LAUNCHER" scripts/test_circle_rl_env.py --urdf "$B2_URDF" --usd model_assets/local/b0_r1_usd/robot.usd --output runs/b2_s0/full_pd_005 --num_envs 2 --headless --device cpu
env FACTORY_VENV="$B2_FACTORY/work/envs/factory-g01" PYTHONDONTWRITEBYTECODE=1 timeout --signal=TERM --kill-after=10s 120s "$B2_LAUNCHER" scripts/test_circle_rl_env.py --urdf "$B2_URDF" --usd model_assets/local/b0_r1_usd/robot.usd --output runs/b2_s0/quick_gravity_001 --quick --num_envs 2 --controller pd_gravity --headless --device cpu
```

测试脚本默认读取 B1-S1 已审核的本地 `runs/b1_s1/plan_005/trajectory.jsonl` 作为传统 IK 诊断输入并校验 hash；换位置时通过 `--trajectory` 显式指定。以上输出目录要求事先不存在。完整命令、环境、输入和逐步结果保留在各输出目录，stdout/stderr 分别为 `runs/b2_s0_full_pd_005.log`、`runs/b2_s0_quick_gravity_001.log`；私有 cooked 顶点和完整日志不上传。原始 B1-S1 运行源代码 hash 对应输入提交；本次仅给其 cooked 几何复用函数新增可选机器人路径参数，旧默认路径不变。

首次 `quick_001` 对两个副本 cooked 顶点做严格逐位相等比较，因约 `1.1e-16` 的浮点差异拒绝；改为绝对容差 `1e-10` 后短测通过。早期 `full_pd_001` 使用相对实测 q 的动作，在普通 PD 零动作下约8步即因重力下垂触发姿态终止；因此改用上一目标增量并公开目标差。`full_pd_002…004` 是目标、奖励或观测仍在修订时的试验，保留本地，不冒充最终31维结果。PhysX cooked 查询 API 的弃用警告保留；它不等于碰撞资产缺失。

## 下一门与学习问题

已具备进入 B2-S1 进行该单一任务的后续环境联调基础，需先审阅奖励截断、CPU 吞吐量、有限时域封装和更多初态；是否启动 PPO 必须另行授权。B0 真实接口仍 BLOCKED。学习验收由研究者作答前保持 NOT_STARTED：

1. 为什么 24 维候选增加了目标差和 episode 时间？圆周起点与终点有什么观测歧义？
2. 为什么零动作保持的是位置目标，而实际末端仍会受普通 PD 重力静差和运动参考影响？
3. 如何从三个奖励项的上下界证明每一步总奖励为正？这为何不能代替碰撞终止？
