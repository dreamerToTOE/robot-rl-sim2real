# B1-S0：Isaac Lab 机器人物理验证

日期：2026-11-13（KST）；输入仓库基线：`f9b8349b0ef14d2bc2e47584d24483156bcd73ca`。

**结论：PARTIAL。** 两次独立试验均完成固定基座 PhysX 加载、静止保持、六轴正负小步和逐步 FK 对照；未观察到非邻接接触、非有限状态或越限。碰撞检测未做人为碰撞的正例注入，PhysX 最终 cooked convex hull 也未单独导出核验，所以不能把“没有收到接触”扩展成整个机器人或工作空间无碰撞。结果仅覆盖本模型、零位邻域和下列 **SIM_ONLY** 数值配置。

## 环境与输入

| 项目 | 本次实际证据 |
|---|---|
| Isaac Sim / Isaac Lab | 4.5.0；Lab Git tag `v2.1.1`，commit `90b79bb2d44feb8d833f260f2bf37da3487180ba`；核心 extension 版本 `0.41.3`。运行时读取而非沿用历史标签，见 [trial 012 metadata](data/B1_S0_TRIAL_012_SUMMARY.json)。 |
| Factory 隔离 venv | Python 3.10.12、PyTorch 2.7.0+cu128、NumPy 1.26.4；Kit 内 SciPy 1.10.1，独立 CPU 筛查进程使用 venv SciPy 1.15.3。运行前后包路径/版本见本地日志；Factory 110 个既有文件 hash 未变，Lab checkout 仍干净。 |
| 仿真 | PhysX CPU、固定步长 1/240 s、TGS、重力 `(0,0,-9.81) m/s²`；headless 启动仍初始化了 RTX 3070 的 Vulkan 图形栈。未执行训练。 |
| 原厂输入 | URDF SHA-256 `78c1b35c65fe747876c19c06c3ca3492d22aeac4416c536f585da494ef6842ae`；使用 B0-R1A 已生成的私有 USD，4 个输入层的 hash 与 [静态导入记录](data/B0_R1_STATIC_IMPORT.json) 一致，试验结束后再次一致。原厂文件未改写。 |
| 场景 | 单台机器人、固定世界基座；无夹爪、吸盘、任务物体、RL 环境、策略、ROS/CAN 或硬件连接。`link6` frame 是临时模型末端，非标定 TCP。 |

配置及来源见 [b1_s0_sim_only.json](../../configs/b1_s0_sim_only.json)：显式 Ideal PD 的 Kp=200 Nm/rad、Kd=4 Nm·s/rad，关节 armature=0.02 kg·m²、摩擦系数=0、effort 上限=20 Nm、PhysX 速度上限=0.5 rad/s；接触偏移 1 mm、rest offset 0，solver 位置/速度迭代 8/4。**全部为 SIM_ONLY，不是硬件规格或部署限制。** armature 是为数值测试加入关节空间的惯性，不改变 CAD 刚体质量/惯量；位置界仍来自 URDF，不代表实机软硬限位。

## 初态、几何与运行时碰撞

`q0=[0,0,0,0,0,0] rad` 在六个模型位置界内；并非因为厂商示例采用零位而选用。独立 CPU 筛查对 q0 的 15 个非邻接连杆对检查 CAD collision mesh 凸包，全部分离；每轴 `[-0.02,0.02] rad` 九点共54个单轴配置，以及 seed `20261113` 的32个组合配置也全部分离。该检查采用凸包半空间相交检验，失败即停止；见 [检查脚本](../../scripts/check_model_collisions.py#L30)。q0 最接近的 `link4/link6` 有 signed intersection inradius `-0.00149861 m`：这个量的绝对值**不是表面净距**。

私有 USD 的7个 authored collision mesh 顶点与 URDF mesh 核对，最大点集 Hausdorff 差 `8.33e-17 m`，双向凸包包含残差不超过 `1.25e-16 m`；该静态核对记录保存在任务工作目录 `work/s0_collision/usd_collision.json`。这不验证 PhysX cooking 后的最终凸包。

相邻关节壳体的 CAD 凸包存在接触或重叠，物理关节默认的直连对排除被保留；不能声称全部21对原始几何无重叠。本次 stage 中7个 collider 均启用且为 convexHull，articulation self-collision 启用，检查到六个关节的 `collisionEnabled=false`；没有发现额外 `FilteredPairsAPI` 目标或 collision group（[运行时检查](../../scripts/validate_sim_robot.py#L157)）。7个刚体均设置零阈值接触报告，订阅 PhysX pair contact，并对每个实际 q 额外做 CPU 凸包分离检查（[代码](../../scripts/validate_sim_robot.py#L184)、[逐步检查](../../scripts/validate_sim_robot.py#L291)）。

两轮各6,240个采样中接触报告均为0、CPU 非邻接凸包均分离；对 trial 012 的 `6,240×15` 个非邻接对复核，全部通过 AABB 分离证书，无需 LP fallback；本次最小 AABB 距离**下界**为 `0.0024068 m`，不是精确的表面最短距离（[脱敏统计](data/B1_S0_COLLISION_SCREENING.json)）。没有为证明报警而故意让机器人碰撞，接触回调的正例触发仍为 UNKNOWN；对动态轨迹的逐步采样也不是连续时间、全工作空间或桌面碰撞证明。

## PhysX 运行时结果

| 检查 | 观察与判定 |
|---|---|
| 加载与固定基座 | **PASS**：7刚体、6个活动 DoF；固定基座；两次最大基座位姿漂移均为0（本次数值精度）。 |
| PhysX DoF 顺序 | **PASS**：`joint1, joint2, joint3, joint4, joint5, joint6`；由 PhysX view 名称核对并按名称映射，而非依赖模型遍历顺序。位置界 readback 与 URDF 比较容差 2e-6 rad。 |
| 质量与惯量 | **PASS（本输入与本后端）**：7体质量与 URDF 对照通过；最大质心位置误差 `5.72e-9 m`，运行时完整惯量矩阵与 USD 主惯量及主轴重建之最大元素差 `2.32e-10 kg·m²`，与 URDF 连杆轴矩阵最大元素差 `4.45e-10 kg·m²`。详细值见 trial metadata。 |
| 稳定性 | **PASS（短时局部）**：各26 s、6,240步，q/dq/effort/FK 有限、位置在模型界内；最大 `|dq|=0.41657 rad/s`，无超过诊断停止阈值 0.55 rad/s 的样本。 |
| 保持及单轴控制 | **PASS（SIM_ONLY）**：2 s hold，六轴分别 `+0.01` 和 `-0.01 rad` 并回零；每轮25个阶段全部完成。详见 [控制报告](B1_S0_JOINT_CONTROL.md)。 |
| 运行时 FK | **PASS（link6、两次各6,240步）**：位置误差最大 `1.9028e-7 m`，姿态误差最大 `5.9057e-7 rad`。详见 [FK 报告](B1_S0_FK_VALIDATION.md)。 |
| 可重复性 | **PASS（同机同环境）**：两轮相同脚本与配置，6,240对时间点的 q、dq、目标、两种命令力矩和关节投影力矩逐项差均为0；不推断跨机器或跨版本逐位确定性。 |

API 文档称 `get_inertias()` 在 COM local frame；本次后端返回的**完整矩阵**经实测已与 USD `principalAxes × diagonalInertia × principalAxesᵀ` 的连杆轴重建相符，原 URDF 的 inertial origin 旋转也已计入。`get_coms()` 的姿态包含主轴方向：若对已重建的运行时矩阵再用它旋转一次，会重复旋转。trial 009 曾因该错误的检查在运动前阻断，保留了失败日志；修正依据是两个独立矩阵重建的数值对照，不能推广为其他后端的通用约定。实现见 [运行时惯量比较](../../scripts/validate_sim_robot.py#L208)；这只验证模型数据传播，不证明 CAD 惯量符合实物。

## 命令、日志和异常

在项目仓库根目录，实际执行的两个命令如下（各自使用新输出目录）：

```bash
env FACTORY_VENV=/home/ubuntu2004/Documents/Codex/2026-10-23/the-github-repository-has-now-been/work/envs/factory-g01 PYTHONDONTWRITEBYTECODE=1 timeout --signal=TERM --kill-after=10s 80s /home/ubuntu2004/Documents/Codex/2026-10-23/the-github-repository-has-now-been/outputs/isaac-factory-repro/scripts/run_sim.sh scripts/validate_sim_robot.py --urdf /tmp/student_ros_b0_audit.sYOXbS/student_ros/src/episode1_urdf_1113/urdf/episode1_urdf_1113.urdf --usd model_assets/local/b0_r1_usd/robot.usd --output runs/b1_s0/trial_012 --headless --device cpu > runs/b1_s0/trial_012.log 2>&1

env FACTORY_VENV=/home/ubuntu2004/Documents/Codex/2026-10-23/the-github-repository-has-now-been/work/envs/factory-g01 PYTHONDONTWRITEBYTECODE=1 timeout --signal=TERM --kill-after=10s 80s /home/ubuntu2004/Documents/Codex/2026-10-23/the-github-repository-has-now-been/outputs/isaac-factory-repro/scripts/run_sim.sh scripts/validate_sim_robot.py --urdf /tmp/student_ros_b0_audit.sYOXbS/student_ros/src/episode1_urdf_1113/urdf/episode1_urdf_1113.urdf --usd model_assets/local/b0_r1_usd/robot.usd --output runs/b1_s0/trial_013 --headless --device cpu > runs/b1_s0/trial_013.log 2>&1
```

两次进程退出码均为0，且有新产生的 `B1_S0_RESULT=PASS` 和 `B1_S0_TIMELINE_STOPPED`；不能仅凭退出码判通过。Kit 的快速关闭会终止进程，`app.close()` 后的 Python 返回标记未观察到。原始逐步数据、完整 stdout/stderr、配置、阶段表、metadata、summary 和退出码保留在忽略目录 `runs/b1_s0/trial_012*`、`trial_013*`；此前的尝试 `trial_001*`–`trial_011*` 也保留。公开的是脱敏后的 [trial 012 摘要](data/B1_S0_TRIAL_012_SUMMARY.json)、[trial 013 摘要](data/B1_S0_TRIAL_013_SUMMARY.json)、[012 约10 Hz 状态摘录](data/B1_S0_TRIAL_012_10HZ.jsonl)、[013 摘录](data/B1_S0_TRIAL_013_10HZ.jsonl)及[重复性比较](data/B1_S0_REPEAT_COMPARISON.json)；完整日志为240 Hz，摘录另保留阶段首尾。

启动调试中，trial 001 因接触报告阈值/instanceable collider 覆盖未按预期应用而在运动前停止，且提前混用两个 SciPy 目录引起扩展导入错误；trial 002 因初始角整数类型造成 Float/Long 不匹配而停止。之后将 CPU 筛查隔离为子进程、对场景 collider 显式设置并 readback、使用浮点 q0。trial 003/004 完成物理步骤后在 Kit 停止回调等待，人工发 SIGTERM；trial 005 在相同关闭等待中被 70 s timeout 终止（124）。回溯定位到 Lab stop 回调等待 GUI 恢复；按 Lab 自身 reset 使用的 guard，仅在结束时绕过该等待，trial 006 及之后的关闭流程可退出。trial 009 的重复旋转检查失败如上所述，均未掩盖。最终两轮日志有 crash reporter 缺失、`rendering_modes` 配置缺失、MaterialX 提示、OmniHub 不可达和旧 dynamic-control 弃用警告；没有本轮 `[Error]` 行。这些警告与历史 B0 材质解析问题均未证明完整视觉渲染通过。

## 阶段门与学习问题

B2 **RL 环境设计的模型与基础仿真输入有条件具备**；在考虑训练前仍需碰撞报告正例、cooked-hull/安装场景核验，以及合理的任务边界。B0 的实机控制接口、真实零位/方向、TCP、负载、速度/加速度/力矩、摩擦与执行器动态仍 UNKNOWN，实机门禁保持 BLOCKED；不得把 SIM_ONLY 限制当成真实安全限制。本任务没有进入 B2、PPO 或实机控制。

1. 为什么每一步比较要用 `body_link_pose_w` 转到 `base_link`，而不是直接把质心 pose 当作 `link6`？
2. 为什么在已经取得完整惯量矩阵后再套一次主轴旋转会得到错误结果？应如何用 USD principal axes 检查自己的坐标假设？

研究者尚未作答，学习验收为 NOT_STARTED。
