# B1-S0：运行时独立 FK 对照

日期：2026-11-13（KST）。**结论：PASS（所测模型和局部轨迹）**。两次独立进程各6,240个物理步，使用实际 q 重算的 `link6` 相对 `base_link` 位姿，与 Isaac Lab 同步读取的刚体 actor 位姿比较，最大位置误差 `1.9027725e-7 m`（约0.1903 µm），最大姿态误差 `5.9057159e-7 rad`（约0.5906 µrad）。这是模型内部数值对照，不能说明真实法兰/TCP 或硬件轨迹误差。

## 方法与参考系

- 模型：未经修改的私有厂商 URDF；其文件 hash、B0-R1A 私有 USD 层 hash 和运行环境见 [物理报告](B1_S0_PHYSICS_VALIDATION.md)。固定基座，临时末端为 `link6` actor/link frame；没有安装工具。
- 名称：从 PhysX view 获取 `joint_names` 与 `body_names`；核对名称集合后，逐个按名称将 q 排入 URDF `joint1…joint6` 顺序，并按名称选择 `base_link`、`link6`（[脚本](../../scripts/validate_sim_robot.py#L195)）。两次实际 DoF 顺序均为 joint1…joint6。
- CPU 侧：从**同一物理步的实际 q** 使用独立 URDF 链 FK 算 `T_base_link6`（[FK 代码](../../scripts/compare_model_fk.py#L127)）。没有拿控制目标 q 代替反馈。
- Isaac Lab 侧：读取 `body_link_pose_w` 的 actor/link 位姿（四元数是 wxyz），以 `inverse(T_world_base) × T_world_link6` 得到同一基坐标系、同一参考点的位姿。没有使用 COM pose；wxyz 转换有独立单元检查（[脚本](../../scripts/validate_sim_robot.py#L39)）。
- 误差：位置为两位置的欧氏范数；姿态为相对旋转角。比较及阈值判断见 [FK 差计算](../../scripts/compare_model_fk.py#L80) 和 [运行时样本](../../scripts/validate_sim_robot.py#L299)。位置/姿态的实验停止阈值分别为 `1e-4 m`、`1e-3 rad`，是 SIM_ONLY 数值门限。

## 结果

两次均包含 2 s 静止保持，以及每轴正负 `0.01 rad` 目标和各自回零；每步均有 FK 计算、误差和碰撞检查，时间范围 `1/240–26 s`。没有用 B0 的静态 `q=0` USD 对照替代运行时比较。

| 指标 | 每轮结果 |
|---|---:|
| 物理步数 | 6,240 |
| 位置误差最大值 | `1.9027725e-7 m` |
| 位置误差 RMS | `1.1461794e-7 m` |
| 姿态误差最大值 | `5.9057159e-7 rad` |
| 姿态误差 RMS | `2.7236881e-7 rad` |
| 基座位置/姿态最大漂移 | `0 m / 0 rad`（本次数值精度） |
| 两轮对应 q/dq/目标/FK 误差最大差 | `0` |

结果分别低于位置和姿态停止门限；因同输入数值的有限精度产生微小残差，不应把该残差解释为物理机器人定位精度。各阶段最大值和逐轴状态见 [trial 012 摘要](data/B1_S0_TRIAL_012_SUMMARY.json)、[trial 013 摘要](data/B1_S0_TRIAL_013_SUMMARY.json)；[重复比较](data/B1_S0_REPEAT_COMPARISON.json)覆盖两个完整日志的6,240对采样。公开的 [012](data/B1_S0_TRIAL_012_10HZ.jsonl) 和 [013](data/B1_S0_TRIAL_013_10HZ.jsonl) 摘录包含 q、目标、速度、力矩、两种末端矩阵与误差；完整 240 Hz 数据和实际命令/警告保留于本地 `runs/b1_s0/trial_012*`、`trial_013*`。

这只验证 URDF 链与同源 USD 在测过的零位邻域及 PhysX 命名下相互一致；它们并非两台独立标定设备。实机编码器零点、轴方向、真实基座安装、法兰/TCP 和负载变形仍 UNKNOWN，不能据此输出真实轨迹误差或实机安全圆轨迹。
