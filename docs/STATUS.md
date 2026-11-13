# 项目状态（事实入口）

**更新时间：2026-11-13（KST）**

**总体状态：`SCAFFOLD_READY / B0_BLOCKED / B1_S0_PARTIAL`**

主线仍是六自由度机械臂无接触空中圆形轨迹跟踪的 Isaac Lab → PPO → 实机 Sim2Real；无相机、夹爪、吸盘或接触任务。

- **B1-S0：PARTIAL。** 复用 Factory 隔离 venv、Isaac Sim 4.5 和 Isaac Lab v2.1.1，对 B0-R1A 私有 USD 两次独立运行相同的 SIM_ONLY 实验。每次26 s、6,240个物理步：固定基座、7刚体、六轴 PhysX 顺序经名称验证；零位保持与六轴正负0.01 rad 单关节目标完成；逐步数据和目标逐项重复一致。最大运行时 link6 FK 位置/姿态误差分别为 `1.9028e-7 m`、`5.9057e-7 rad`。无非邻接接触报告、CPU 凸包交叠、NaN、模型位置越限或设定异常速度。碰撞报警正例、PhysX cooked hull 和完整安装场景未验证，因此不将整个阶段标记为 PASS。
- 初始零位来自位置界与 CPU 凸包筛查；15个非邻接连杆对检测启用，标准直连关节对排除保留，原 CAD 凸包的相邻壳体存在重叠。该结果不构成真实碰撞净空证据。完整惯量矩阵、质心、质量和 USD 主惯量重建对照通过，额外 armature 与 CAD 惯量分开。
- Kp、Kd、armature、摩擦、速度/力矩上限、仿真时间步和停止阈值均为 **SIM_ONLY**，不是厂商参数或真实部署安全值。`link6` 只是暂定模型参考点，真实法兰/TCP 未标定。
- B0-R1A 环境及模型审查保持 **PARTIAL**：Lab commit `90b79bb2d44feb8d833f260f2bf37da3487180ba`；URDF/SDF `q_sdf=[q1,-q2,-q3,-q4,q5,q6]` 后 joint2 origin 仍存在约0.5 mm 基座系偏差，限位也不同。不能据此推断实机轴方向。原厂 URDF 仍为候选 canonical；原厂文件与本地派生 USD 不提交。
- B0 实机门禁保持 **BLOCKED**：厂商 `dynamic_move_with_velocity` 的目标覆盖/排队、时序、停止、实时反馈和实际闭环能力未验证。真实型号、零位/编码器映射、软硬限位、速度/加速度/力矩、传动/摩擦、安装/负载与 TCP 等资料仍缺失。
- 本次没有创建 RL 环境、启动 PPO、连接或驱动实机；没有改变 Factory 源码或选定环境文件（110个 hash 未变），没有上传厂商模型、mesh、SDK 或派生 USD。成功及失败尝试的完整日志保留在本地忽略目录 `runs/b1_s0/`。

| 门 | 状态 |
|---|---|
| B0 实机接口与安全 | `BLOCKED` |
| B0-R1A 环境与静态模型 | `PARTIAL` |
| B1-S0 物理和基础控制 | `PARTIAL`（局部物理/控制/FK通过；碰撞报警正例及 cooked hull 未核实） |
| B1 其余工作 | `NOT_STARTED` |
| B2–B7、PPO、实机部署 | `NOT_STARTED` |
| 用户学习验收 | `NOT_STARTED` |

本次报告与脱敏证据：

- [物理验证、输入、命令与异常](reports/B1_S0_PHYSICS_VALIDATION.md)
- [静止保持和单关节控制](reports/B1_S0_JOINT_CONTROL.md)
- [动态 link6 FK 对照](reports/B1_S0_FK_VALIDATION.md)
- [两次重复性统计](reports/data/B1_S0_REPEAT_COMPARISON.json)、各次 10 Hz 状态摘录及逐阶段摘要；完整240 Hz数据仅保留本地。
- `scripts/validate_sim_robot.py`、`scripts/check_model_collisions.py`、SIM_ONLY 配置及 CPU 单元测试。
- 既有 [B0-R1A 环境审查](reports/B0_R1_ENVIRONMENT_AUDIT.md)、[URDF/SDF 比较](reports/B0_R1_URDF_SDF_FK_COMPARISON.md) 和 [canonical 模型计划](reports/B0_R1_CANONICAL_MODEL_PLAN.md) 继续有效。

**下一决策门：** 这些结果已为 B2 的 RL 环境设计提供条件，但不自动进入 B2 或训练。后续先解决碰撞报告正例和 cooked-hull/安装场景验证，再根据实际机器人资料确定可行工作空间和动作约束；实机前仍须按 `docs/SAFETY_GATE.md` 完成独立审批与只读/影子验证。理解问题保留在物理报告中，研究者尚未作答。
