# 项目状态（事实入口）

**更新时间：2026-11-14（KST）**

**总体状态：`SCAFFOLD_READY / B0_BLOCKED / B1_S1_PASS_SIM_ONLY / B2_S0_PASS_SIM_ONLY / B2_S1_PARTIAL_SIM_ONLY`**

主线仍是六自由度机械臂无接触空中圆形轨迹跟踪的 Isaac Lab → PPO → 实机 Sim2Real；无相机、夹爪、吸盘或接触任务。

- **B2-S0：PASS（限定 SIM_ONLY 两环境测试）。** 已实现 Isaac Lab `DirectRLEnv`，31维观测、6维相对上一目标的位置增量、240 Hz物理/60 Hz控制；在 q_home 附近执行碰撞和奇异性筛查后 reset，参考覆盖加速、完整一圈、减速与保持，单 episode 10.5 s。额外观测包含目标差与 episode 时间，避免控制器状态和参考时序歧义。
- 默认普通PD下，传统 IK 诊断驱动完成两个630步 episode，恒速圆 RMSE 约3.406 mm；零动作走满上限但全段RMSE约14.29/14.02 mm。随机动作两次首个 episode 分别在56/102步因姿态偏差终止；共630次批量控制步内14次姿态终止。碰撞、越限、非有限值故障注入和双环境隔离通过，同种子32步最大差0。详见 [测试汇总](reports/data/B2_S0_TEST_RESULTS.json)。
- B2-S0 的旧 clipped quadratic 奖励和历史证据保留不变；位置误差以5 mm归一化后截断的问题由 B2-S1 单独审查。独立 `pd_gravity` 的 B2-S0 短测不覆盖本阶段结论。
- **B2-S1：PARTIAL（SIM_ONLY，未训练）。** 零动作/随机/传统 IK 的 `e≤5 mm` 比例分别为16.90%/26.98%/100%；新增 `exp_linear` 候选 `r_pos=(0.03+exp(-e/0.005))/630`，在5 mm以上保持区分能力。候选配置下普通PD和 `pd_gravity` 均完成2环境、630步完整回归；传统IK恒速圆 RMSE/P95/最大误差分别为 `3.406/3.708/3.712 mm` 与 `0.076/0.077/0.077 mm`，均通过冻结成功判据。姿态终止门 `0.05 rad` 保持不变。1/2/4环境 CPU 吞吐量为 `38.45/37.87/42.89 env-step/s`；未测GPU、未启动PPO。
- **B1-S1：PASS（所测 SIM_ONLY 传统基线）。** 复用同一环境和私有 USD，完成4,320步/18秒全序列，恒速圆5–13秒：圆心 `(0.3300382964,0.0070540661,0.4020848552) m`、半径10 mm、周期8 s、固定link6姿态目标。普通PD圆周RMSE/P95/最大误差为 `3.4073/3.7142/3.7182 mm`；单独保留的模型重力前馈对照为 `0.124385/0.124860/0.124904 mm`。两种控制均完成一圈，无碰撞/越限/NaN/饱和，逐步实际q、速度、加速度、力矩、真实仿真时钟及独立FK数据已保存。
- **B1-S1 碰撞验证：PASS（所测场景）。** 修复Lab context构造时全局接触处理被关闭的问题；合成立方体正例有24条报告，机器人折叠正例第1步报告base_link/link6等非邻接碰撞，正常零位负例60步为0。7个runtime cooked hull已取出，证实其简化/外扩差异；采用CAD+cooked外包络与每体1 mm接触余量，在运动前及每个实际q上筛查。保留self-collision和标准直连关节对排除，无新增全局过滤。
- **B1-S0 历史状态仍为PARTIAL。** 基础保持/六轴小步和动态FK数据保留；B1-S1复核发现S0的零回调不能作为有效碰撞负例证据，已补勘误。S1补齐了局部碰撞正例及cooked几何验证；不追溯改写旧运行结果。完整工作空间、连续扫掠体、真实桌面/安装、相邻CAD壳体接触仍未验证。
- Kp、Kd、armature、摩擦、速度/力矩上限、仿真时间步和停止阈值均为 **SIM_ONLY**，不是厂商参数或真实部署安全值。`link6` 只是暂定模型参考点，真实法兰/TCP 未标定。
- B0-R1A 环境及模型审查保持 **PARTIAL**：Lab commit `90b79bb2d44feb8d833f260f2bf37da3487180ba`；URDF/SDF `q_sdf=[q1,-q2,-q3,-q4,q5,q6]` 后 joint2 origin 仍存在约0.5 mm 基座系偏差，限位也不同。不能据此推断实机轴方向。原厂 URDF 仍为候选 canonical；原厂文件与本地派生 USD 不提交。
- B0 实机门禁保持 **BLOCKED**：厂商 `dynamic_move_with_velocity` 的目标覆盖/排队、时序、停止、实时反馈和实际闭环能力未验证。真实型号、零位/编码器映射、软硬限位、速度/加速度/力矩、传动/摩擦、安装/负载与 TCP 等资料仍缺失。
- 本次没有注册或启动 PPO 训练，也没有连接或驱动实机；没有改变 Factory 源码或选定环境文件（110个 hash 未变），没有上传厂商模型、mesh、SDK、派生 USD 或 cooked 顶点。成功及失败尝试的完整日志保留在本地忽略目录 `runs/b1_s0/`、`runs/b1_s1/`、`runs/b2_s0/`。

| 门 | 状态 |
|---|---|
| B0 实机接口与安全 | `BLOCKED` |
| B0-R1A 环境与静态模型 | `PARTIAL` |
| B1-S0 物理和基础控制 | `PARTIAL`（历史记录；碰撞回调勘误见S1） |
| B1-S1 无策略圆轨迹基线 | `PASS`（SIM_ONLY，限定模型/轨迹/数值诊断） |
| B1 其余工作 | `NOT_STARTED` |
| B2-S0 圆轨迹 RL 环境 | `PASS`（SIM_ONLY，两环境、无训练测试） |
| B2-S1 可训练性与 PPO 前检查 | `PARTIAL`（SIM_ONLY，候选 reward 与完整回归通过；无PPO） |
| B3–B7、PPO、实机部署 | `NOT_STARTED` |
| 用户学习验收 | `NOT_STARTED` |

本次报告与脱敏证据：

- [RL 环境设计](reports/B2_S0_RL_ENV_DESIGN.md)、[观测和动作](reports/B2_S0_OBSERVATION_ACTION.md)、[奖励、reset 与测试](reports/B2_S0_REWARD_RESET_TESTS.md)。
- [B2-S1 奖励审查](reports/B2_S1_REWARD_AUDIT.md)、[终止和动作审查](reports/B2_S1_TERMINATION_AUDIT.md)、[训练准备和吞吐量](reports/B2_S1_TRAINING_READINESS.md)。
- SIM_ONLY 环境与配置、脱敏数值汇总、43项 CPU 单元测试；完整逐步日志及派生顶点仅保留本地。
- [碰撞正/负例和cooked hull](reports/B1_S1_COLLISION_VALIDATION.md)
- [离线圆轨迹、IK与完整过渡](reports/B1_S1_CIRCLE_TRAJECTORY.md)
- [普通PD和重力前馈对照、指标、图及命令](reports/B1_S1_CLASSICAL_BASELINE.md)
- 脱敏完整阶段摘要、约10 Hz状态摘录、XY/误差时间图；完整240 Hz数据和派生顶点仅保留本地。
- 最小离线规划/执行/评估脚本与 SIM_ONLY 配置。
- 既有 [S0物理记录及勘误](reports/B1_S0_PHYSICS_VALIDATION.md)、[控制](reports/B1_S0_JOINT_CONTROL.md)和[FK](reports/B1_S0_FK_VALIDATION.md)保留。
- 既有 [B0-R1A 环境审查](reports/B0_R1_ENVIRONMENT_AUDIT.md)、[URDF/SDF 比较](reports/B0_R1_URDF_SDF_FK_COMPARISON.md) 和 [canonical 模型计划](reports/B0_R1_CANONICAL_MODEL_PLAN.md) 继续有效。

**下一决策门：** B2-S1 已完成 PPO 前环境审查，但当前仍只具备小规模 SIM_ONLY smoke 的条件；GPU 吞吐量、PPO runner 的有限时域 bootstrap、更多初态和真实接口仍待验证。PPO 训练需明确授权，不自动进入 B3。更广工作空间、实际硬件接口和安装场景仍需验证，实机前仍须按 `docs/SAFETY_GATE.md` 完成审批与只读/影子验证。理解问题见 B2-S0/B2-S1 报告，研究者尚未作答。
