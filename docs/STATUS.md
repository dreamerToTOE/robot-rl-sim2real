# 项目状态（事实入口）

**更新时间：2026-11-13（KST）**
**状态：`SCAFFOLD_READY / B0_BLOCKED`**

**本次 B0-R1A：`PARTIAL`；环境最小启动 `PASS`，静态模型转换结构检查 `PASS`，动态/实机门禁 `BLOCKED`。**

- 已确定：主线是六自由度机械臂**无接触空中圆形轨迹跟踪**；PPO；没有相机/夹爪/吸盘；目的为学习完整 Sim2Real。
- 已交付：目录、研究约束、只读 Codex 指令、纯数学参考轨迹生成器和 Python 单元测试。
- 已完成：本地私有 `student_ros.zip` 的只读静态审查；压缩包 SHA-256 为 `bbc42d6efb76aeb7eea48897d231b5bc3a20d0e081f08816722cd048872d5d4e`。报告见 `docs/reports/B0_MODEL_AND_SDK_AUDIT.md`、`B0_INTERFACE_MATRIX.md`、`B0_SIM2REAL_FEASIBILITY.md`。
- B0-R1A 环境：复用既有 Factory `factory-g01` 隔离环境、Isaac Lab v2.1.1（`90b79bb2d44feb8d833f260f2bf37da3487180ba`）与 Isaac Sim 4.5。空场景3次 Kit update、0次物理步通过；Factory 核心包版本未变，110个选定配置/元数据/启动文件 hash 未变。旧 Conda `isaaclab` 的失效 editable 保持原样，不再作为本任务入口。
- B0-R1A 模型：纯 CPU 共307配置（283共同合法域+24原生限位样本）验证 `q_sdf=[q1,-q2,-q3,-q4,q5,q6]`。相同 base 下仍有 joint2 origin 导致的约0.5 mm Z偏差；固定基坐标注册后末端数值一致，但 base_link/link1 错开，且限位不一致，不能把两模型或实机方向等同。
- B0-R1A 静态导入：原厂 URDF → 本地私有 USD 已完成，7刚体/6 revolute/14 mesh、世界固定 articulation root、轴/零位/惯量对照通过。timeline 未播放、无关节目标、0物理步。3个 `OmniPBR.mdl` 材质引用的通用解析告警和一次关闭路径段错误已保留，整体 `PARTIAL`。
- 尚未完成：运行时 PhysX DoF/动力学、完整渲染和碰撞验证、任何 RL 环境、训练、策略评估、影子推理、实机运动。未进入 B1 运动控制测试。
- 机器人型号：压缩包目录名为 `episode1_urdf_1113`，只能确认包内模型名称；实物型号、安装、法兰/TCP、编码器映射尚未被 B0 实机/厂商材料确认。
- 模型结论：推荐保留原厂 URDF 为候选 canonical；axis 文本差异不能独立判定 FK 不一致，joint6 两模型同为 +Z（已更正旧报告）。effort/velocity 全为0，速度/加速度/力矩/摩擦/伺服动态 UNKNOWN，不能用于动态训练或实机安全限幅；临时参考点只用 link6 frame，真实法兰/TCP待标定。
- 厂商压缩包：**不提交到仓库**，版权/再分发、真实动态参数和二进制 SDK 行为仍需确认。
- SDK 结论：ROS 表面接口和二进制方法已定位；`dynamic_move_with_velocity` 的覆盖、排队、周期、停止和真实状态语义 UNKNOWN，不能称作 PPO 在线闭环。
- 策略结果：**不存在**。

| 门 | 状态 |
|---|---|
| B0 | `BLOCKED`（静态审查完成；动态/硬件放行证据缺失） |
| B0-R1A | `PARTIAL`（可复用环境和静态转换已验证；模型/材质/硬件未知仍保留） |
| B1–B7 | `NOT_STARTED` |
| 工程最终验收 | `NOT_STARTED` |
| 用户理解验收 | `NOT_STARTED` |

本次交付：

- [环境恢复审查](reports/B0_R1_ENVIRONMENT_AUDIT.md)
- [URDF/SDF FK 数值比较](reports/B0_R1_URDF_SDF_FK_COMPARISON.md)
- [canonical 模型和静态导入计划/结果](reports/B0_R1_CANONICAL_MODEL_PLAN.md)
- `scripts/compare_model_fk.py`、`scripts/audit_vendor_archive.py`、两个静态 smoke 脚本及8个合成 FK 测试；CPU单元测试12/12通过。脱敏数值证据在 `docs/reports/data/B0_R1_*.json`，厂商资产及派生 USD 没有提交。

**下一决策门：** 用户审阅本次报告，并提供准确型号/模型版本、joint2高度尺寸依据、六轴机械零位/编码器映射与减速比、真实软硬限位和动态规格、法兰/TCP图与安装/桌面/负载资料、SDK目标覆盖/时序/停止说明。B1静态模型导入的输入条件已具备；B1运动测试、PPO及实机控制仍须另行授权，不自动跨门。后续厂商模型仅从本地私有路径读取。
