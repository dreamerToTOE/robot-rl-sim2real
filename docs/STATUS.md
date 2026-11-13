# 项目状态（事实入口）

**更新时间：2026-11-13（KST）**
**状态：`SCAFFOLD_READY / B0_BLOCKED`**

- 已确定：主线是六自由度机械臂**无接触空中圆形轨迹跟踪**；PPO；没有相机/夹爪/吸盘；目的为学习完整 Sim2Real。
- 已交付：目录、研究约束、只读 Codex 指令、纯数学参考轨迹生成器和 Python 单元测试。
- 已完成：本地私有 `student_ros.zip` 的只读静态审查；压缩包 SHA-256 为 `bbc42d6efb76aeb7eea48897d231b5bc3a20d0e081f08816722cd048872d5d4e`。报告见 `docs/reports/B0_MODEL_AND_SDK_AUDIT.md`、`B0_INTERFACE_MATRIX.md`、`B0_SIM2REAL_FEASIBILITY.md`。
- 尚未完成：Isaac Lab 导入、任何 RL 环境、训练、仿真策略评估、影子推理、实机运动；当前 Isaac Lab 环境 import 失败。
- 机器人型号：压缩包目录名为 `episode1_urdf_1113`，只能确认包内模型名称；实物型号、安装、法兰/TCP、编码器映射尚未被 B0 实机/厂商材料确认。
- 模型结论：URDF/SDF 都是 7 link/6 revolute 的静态链，但 joint 轴/范围存在差异，effort/velocity 全为 0，不能直接用于动态训练或安全限幅。
- 厂商压缩包：**不提交到仓库**，版权/再分发、真实动态参数和二进制 SDK 行为仍需确认。
- SDK 结论：ROS 表面接口和二进制方法已定位；`dynamic_move_with_velocity` 的覆盖、排队、周期、停止和真实状态语义 UNKNOWN，不能称作 PPO 在线闭环。
- 策略结果：**不存在**。

| 门 | 状态 |
|---|---|
| B0 | `BLOCKED`（静态审查完成；动态/硬件放行证据缺失） |
| B1–B7 | `NOT_STARTED` |
| 工程最终验收 | `NOT_STARTED` |
| 用户理解验收 | `NOT_STARTED` |

**下一阶段入口：** `docs/CODEX_B0_TASK.md` 与三份 B0 报告中的决策门。后续模型文件仍只允许从用户本地私有路径读取，任何进度变更都必须附真实证据。
