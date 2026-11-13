# B1-S1：碰撞检测正例、负例与 cooked hull 验证

日期：2026-11-13（KST）；输入基线：`a8f1ba99d903ed8f7e3284c0eb63aa6ca29b6199`。

**结论：PASS（所测仿真检测链路）。** 合成碰撞正例、机器人非邻接自碰撞正例和正常零位负例通过；实际运行时的7个 cooked convex hull 已取出并比较。此结论覆盖规定的采样测试，不构成连续时间、全工作空间或实机碰撞安全认证。

## S0 缺陷与修复

B1-S0 给7个刚体添加了零阈值 `PhysxContactReportAPI`，但遗漏了全局接触处理开关。所用 Lab v2.1.1 的 `source/isaaclab/isaaclab/sim/simulation_context.py:274` 在构造 `SimulationContext` 时把 `/physics/disableContactProcessing` 设为 `True`。物理碰撞求解仍会发生，Python 接触回调却不会产生预期数据。探针实测：动态立方体确实发生位移/碰撞响应，但开关读回 `True`，回调为空；仅将启用操作移到 context 构造之后，立即收到接触。

修复位置：[正例脚本](../../scripts/validate_collision_positive.py#L52)、[圆轨迹执行](../../scripts/run_circle_trajectory.py#L239)及既有 `scripts/validate_sim_robot.py`。均在 context 创建后启用，并在 reset 后再次读回检查。**S0 历史日志的“零接触回调”不再视为有效负例证据**；其独立 CPU 几何/FK 数据仍保留。已在 [S0 物理报告](B1_S0_PHYSICS_VALIDATION.md)补充勘误。

## 正例与负例

| 试验 | 本地输出 | 判据与结果 |
|---|---|---|
| 合成立方体 | `runs/b1_s1/collision_control_010/` | 重力下动态立方体落向运动学立方体；120步收到24条正例报告，首条 separation 为 `-0.0153347 m`；独立分离对为0条。PASS。 |
| 机器人自碰撞正例 | `runs/b1_s1/collision_robot_positive_011/` | 在独立场景初始化一个位置界内的折叠姿态；第1步收到 `link6/base_link`、`link5/base_link`、`link4/base_link` 报告，立即结束。预期非邻接 `base_link/link6` 的 separation 约 `-0.0322…-0.0335 m`。PASS。 |
| 机器人零位负例 | `runs/b1_s1/collision_robot_negative_009/` | q=0、零重力的独立保持场景，60步、0条报告。PASS。 |
| 正式圆轨迹 | `runs/b1_s1/trial_005/`、`trial_006_gravity/` | 正常重力，各4,320步；接触处理已启用，非邻接几何检查逐步执行，无报告或几何报警。PASS。 |

机器人正例的数值输入见 [SIM_ONLY fixture](../../configs/b1_s1_collision_sim_only.json)，来源是离线合法域搜索，CPU 的 `base_link/link6` CAD/cooked 凸包相交内切半径证据约 `+0.0125 m`。这是**有意碰撞的测试夹具**，不是进入轨迹的姿态或实机目标。没有运行随机策略。合成正例验证全局回调，机器人正例进一步验证实际 USD 的非邻接自碰撞报告。

所有机器人测试均保留7个 collider、启用 articulation self-collision、保留标准直连关节对排除；未添加 `FilteredPairsAPI` 目标或 collision group。没有通过删除碰撞体或过滤全部对来通过。数值证据见 [碰撞汇总](data/B1_S1_COLLISION_VALIDATION.json)。

## cooked hull 与源几何

使用本机 Isaac Sim 4.5 的 `PhysXCooking.get_nb_convex_mesh_data/get_convex_mesh_data`，在运行中查询 `/World/Robot/<link>/collisions`。本机绑定声明位于 `extsPhysics/omni.physx/omni/physx/bindings/_physx.pyi:2005`；该 API 在此版本已弃用但仍成功返回数据，警告保留。查询的是实际 runtime representation，未把 USD 上的 `convexHull` 字符串当成验证。顶点经 collider→link 相对变换后与 URDF collision mesh 的凸包比较，见 [实现](../../scripts/run_circle_trajectory.py#L37)。

| 连杆 | CAD凸包顶点 | cooked顶点 | cooked越出CAD平面的最大值 mm | CAD越出cooked平面的最大值 mm |
|---|---:|---:|---:|---:|
| base_link | 94 | 39 | 1.7931 | 0.3950 |
| link1 | 400 | 40 | ≈0 | 2.4400 |
| link2 | 438 | 34 | ≈0 | 2.1257 |
| link3 | 573 | 37 | ≈0 | 2.6112 |
| link4 | 502 | 34 | ≈0 | 1.7294 |
| link5 | 835 | 43 | ≈0 | 1.9365 |
| link6 | 540 | 36 | ≈0 | 1.3365 |

每连杆恰好一个 runtime convex hull。上表是单位法向平面的带符号包含残差，**不是 Hausdorff 距离或碰撞净距**。cooking 会简化形状；base_link 还有向外的差异。因此，不能假设源 CAD 凸包必然包住实际 cooked hull。原始 mesh 的凹部同样不由单凸包表示。

正式执行前，程序取每连杆 CAD 与 cooked 顶点并集的凸包，再将各平面向外移1 mm，AABB也扩展1 mm，保守覆盖各体接触 skin。先检查全部4,320个参考配置：`4,320×15=64,800` 个非邻接对，3,019次 LP fallback，全部分离；运行时对实际反馈 q 做同样检查并同时监听 PhysX。见 [几何计算](../../scripts/check_model_collisions.py#L31)和[正式数值记录](data/B1_S1_BASELINE_SUMMARY.json)。AABB 重叠时 LP 仍可证明分离，此时仅记录平凡距离下界0，不把0解释为接触。

派生顶点仅保存在各本地 run 的 `private_cooked_vertices.json`，不提交；公开汇总只含顶点数、包含残差和体积等统计。厂商 URDF、mesh、SDK、私有 USD 均未改写或上传。

## 失败保留及边界

- `collision_control_001…003` 均未触发正例；`runs/b1_s1/debug/control_004` 读回开关为 True，`control_005` 仅调整启用时机后通过。这些失败没有被退出码0掩盖。
- `collision_robot_negative_008` 在运动前因整数初态造成 Float/Long 不匹配而停止，统一转换为浮点后 `_009` 通过。
- 旧 SciPy/HiGHS 在若干几乎平行的半空间上返回 status 4；脚本严格拒绝该结果。本次将求解可行性容差设为 `1e-9`，仍保留 success、有限值和残差检查；不确定配置不作为通过证据。合成相交/分离/膨胀几何有 CPU 测试。
- 本次没有证明任意姿态、相邻壳体原始几何、连续扫掠体或真实安装场景无碰撞；没有桌面或任务物体。实机安装与自碰撞模型可信度仍 UNKNOWN。

环境、命令、原始日志索引及版本见 [基线报告](B1_S1_CLASSICAL_BASELINE.md)和[运行证据](data/B1_S1_RUNTIME_EVIDENCE.json)。下一门是另行授权的 B2 仿真环境构建，不自动执行。
