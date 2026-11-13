# B0-R1A 候选 canonical 模型与最小导入计划

审查日期：2026-11-13（KST）。范围：厂商模型只读 CPU 检查、候选模型决策和无运动的导入方案；未运行 PPO、SDK、机器人或任何运动指令。

## 1. 决策与证据边界

**候选 canonical：以本地厂商 URDF 为唯一运动学基线，状态 PARTIAL。** 保留厂商关节名称、顺序、origin、axis、位置界、质量、CoM 和完整惯量；不根据 SDF 的轴文本修改 URDF，不根据本次模型对比猜测实机电机方向。该决策适用于下一次静态导入核验，尚不代表模型已经匹配实物或可用于动态训练。

输入来自私有 `student_ros.zip`，本报告的厂商路径均是压缩包内相对路径。未上传、修改或重打包厂商文件。

| 输入 | SHA-256 |
|---|---|
| `student_ros.zip` | `bbc42d6efb76aeb7eea48897d231b5bc3a20d0e081f08816722cd048872d5d4e` |
| `student_ros/src/episode1_urdf_1113/urdf/episode1_urdf_1113.urdf` | `78c1b35c65fe747876c19c06c3ca3492d22aeac4416c536f585da494ef6842ae` |
| `student_ros/src/episode1_urdf_1113/urdf/robot.sdf` | `352f42d261cb4e17590dae1e36a7d06c5db19978b85ed757aefdff92a69cde26` |

后文用 `U` 指代上述 URDF，用 `S` 指代上述 SDF。数据来自 XML 解析与 STL 二进制只读解析；惯量特征值检查使用当前系统 Python 3.10.12 的既有 NumPy 1.24.4，不安装依赖。数值表保留足够审查精度，实际适配必须保留原始字段精度，不能抄取表中显示舍入值生成模型。

本次 CPU 形态检查的复现命令（从仓库根目录执行，`<私有压缩包路径>` 为本机输入）为 `python3 scripts/audit_vendor_archive.py <私有压缩包路径> > docs/reports/data/B0_R1_MODEL_STATISTICS.json`。退出码 0，脱敏数值结果见 [模型统计](data/B0_R1_MODEL_STATISTICS.json)。自研脚本不解压、不导入 SDK，通过 assert 检查原始 hash 不变、7 组质量/惯量、14 份 STL 的格式、有限坐标和同名文件一致性；未向基础包增加 NumPy 依赖。提交统计不含私人文件路径、原 XML、顶点、面片或 SDK。

| 检查项 | 状态 | 含义 |
|---|---|---|
| 单一根、完整 6R 串联链、父子关系 | PASS | 对归档内模型的静态检查 |
| 7 组质量、CoM、惯量数值合法性 | PASS | 正质量、惯量正定且满足主惯量三角不等式；不是称重/辨识 |
| 14 个 visual/collision 文件可读、结构与有限坐标检查 | PASS | 文件完整性；不是碰撞安全验证 |
| URDF 与 SDF 对应关系 | PARTIAL | 存在模型角度符号映射；完整 FK 差异见专门报告 |
| 候选 canonical 选择 | PARTIAL | 选 URDF 为后续验证基线；真实几何、零位与方向仍需核验 |
| 真实速度、加速度、力矩、伺服动态 | UNKNOWN | 无可靠硬件参数证据 |
| USD 静态结构核验 | PASS（限定范围） | 本次完成转换、7 刚体/6 revolute、固定 articulation root、零位/轴/限位/惯量/14 mesh 检查 |
| USD 导入总体与最终 articulation 配置 | PARTIAL | MDL 材质解析告警；未创建 PhysX articulation view，未验证动力学/碰撞 |
| 动态仿真、实机或 B1 运动放行 | BLOCKED | 本次没有授权，也没有相应验证证据 |

## 2. 完整运动学参数表

拓扑：`base_link → joint1 → link1 → joint2 → link2 → joint3 → link3 → joint4 → link4 → joint5 → link5 → joint6 → link6`。7 个 link、6 个 revolute joint；没有 fixed、mimic、floating joint 或分叉。`base_link` 是模型根，原始 URDF 不含 `world` 固定连接。

坐标约定：origin 表示父 link 到关节零位 frame 的变换；`RPY` 按 `Rz(yaw) Ry(pitch) Rx(roll)` 组成，axis 位于 joint frame，正角按该轴右手定则。所有长度为 m，角度为 rad。**这只定义模型角度，不定义电机编码器角度。**

| 顺序/名称 | parent → child | origin xyz | origin rpy | axis | 位置下界 / 上界 | U 行号 |
|---|---|---|---|---|---|---|
| 1 / `joint1` | `base_link → link1` | `0 0 0` | `0 0 0` | `0 0 1` | `-3.141592654 / 2.792526803` | 87–104 |
| 2 / `joint2` | `link1 → link2` | `0.055 -0.02 0.166` | `1.5708 0 0` | `0 0 -1` | `-1.570796327 / 1.570796327` | 145–162 |
| 3 / `joint3` | `link2 → link3` | `0 0.2 0` | `0 0 3.1416` | `0 0 -1` | `-1.448623279 / 1.396263402` | 203–220 |
| 4 / `joint4` | `link3 → link4` | `0 -0.056 -0.02` | `-1.5708 0 -1.5708` | `0 0 -1` | `-0.523598776 / 5.323254219` | 261–278 |
| 5 / `joint5` | `link4 → link5` | `0 0 -0.192` | `1.5708 -1.5708 0` | `0 0 -1` | `-1.919862177 / 1.919862177` | 319–336 |
| 6 / `joint6` | `link5 → link6` | `-0.055 0 0` | `1.5708 0 -1.5708` | `0 0 1` | `-0.523598776 / 5.323254219` | 377–394 |

`q=[0,0,0,0,0,0]` 位于所有 URDF 声明区间内；模型零位由 origin 确定，不要求六个 link frame 对齐，也不代表实机上电/回零/安全初态。程序常量角度不是校准记录。编码器到 canonical 角度的 `q_model = sign × (q_encoder - offset)`（及顺序、单位、可能的比例）目前全部等待实物/厂商确认，不能拿 URDF/SDF 的符号映射替代。

厂商 URDF 的 origin 使用 `1.5708/3.1416` 等截断小数。本阶段原样保留；无证据不得“修圆”为精确 π 或 π/2。每轴 `effort=0, velocity=0` 均按无有效动态限制处理，不把 0 解释成允许的真实运行速度或力矩。URDF 位置界同样仅是模型声明，尚未通过硬件限位核验。

### 对上一轮静态文字结论的更正

上一轮 `B0_MODEL_AND_SDK_AUDIT.md` 第 4 节及 `B0_SIM2REAL_FEASIBILITY.md` 风险条目把 `joint6` 列入“反轴”范围；实际 `U:387–388` 与 `S:318–319` 都是 `+Z`。局部轴文本相反的只有 `joint2/3/4`。`joint5` 的欧拉角写法不同不能直接说明旋转矩阵不同，`joint3` 的角度相差约 2π 也不能直接说明零位不同。以 [FK 数值比较报告](B0_R1_URDF_SDF_FK_COMPARISON.md) 为本次判断依据。

确定存在的独立字段差异包括 `joint2` 平移高度：URDF `0.166` m（`U:149`），SDF `0.1655` m（`S:95`）；SDF 所有位置界都是 `[-3,3]`，与 URDF 的逐轴界不同。候选 canonical 不把这 0.5 mm 改成平均值，也不采用两者更宽的区间；先保留 URDF，并请求厂商说明版本来源。

## 3. 质量、质心和完整惯量

所有 inertial origin 的 `rpy=0 0 0`。URDF 与 SDF 的 7 组质量、CoM、inertial rpy 和六项惯量逐字段转为双精度浮点后完全相同（最大绝对差 0），不把文本浮点格式差异当作参数差异。下表 CoM 位于对应 link frame，惯量为质心处 inertial frame 的对称张量，单位 kg·m²。矩阵使用原始六项组成：

`I = [[Ixx,Ixy,Ixz],[Ixy,Iyy,Iyz],[Ixz,Iyz,Izz]]`。

| link | 质量 kg | CoM xyz（m，显示舍入） | U 行号 |
|---|---|---|---|
| `base_link` | 1.94434524582 | `0.001921778812 -1.350890629e-11 0.05721571081` | 9–22 |
| `link1` | 0.623226472029 | `0.01294937287 0.004841920696 0.1428562942` | 49–62 |
| `link2` | 0.569588734459 | `-1.328698617e-6 0.1000016653 0.01595972324` | 107–120 |
| `link3` | 0.549826349800 | `-0.005030275768 -0.04048871085 -0.01993881307` | 165–178 |
| `link4` | 0.396344467834 | `2.881181013e-7 -5.276423967e-5 -0.1325931790` | 223–236 |
| `link5` | 0.151750313627 | `-0.008047529931 3.799627568e-8 1.245749666e-7` | 281–294 |
| `link6` | 0.0182966455302 | `0 -3.286579022e-6 -0.008657029031` | 339–352 |

| link | Ixx | Iyy | Izz | Ixy | Ixz | Iyz |
|---|---|---|---|---|---|---|
| `base_link` | 0.00529698934 | 0.00502085388 | 0.00571194526 | 6.54015849e-13 | 0.000148912654 | -1.11673711e-13 |
| `link1` | 0.000786968431 | 0.00116761049 | 0.00135422216 | 3.99405015e-5 | -0.000207598516 | -3.06673432e-6 |
| `link2` | 0.00316756336 | 0.000267820703 | 0.00333864892 | -1.66564953e-9 | 1.17608929e-8 | -1.47208629e-8 |
| `link3` | 0.000620878135 | 0.000655622275 | 0.00102866996 | -5.54581979e-5 | -4.17229442e-7 | -1.61868183e-7 |
| `link4` | 0.000695803456 | 0.000582583035 | 0.000289626955 | 1.34035755e-10 | -1.05098389e-9 | 9.30436603e-9 |
| `link5` | 6.52695661e-5 | 6.74351951e-5 | 7.16896044e-5 | 1.38019799e-10 | 4.08040033e-10 | -9.14692974e-12 |
| `link6` | 2.75581662e-6 | 2.75621230e-6 | 4.78464023e-6 | 4.25839601e-14 | 8.47285649e-22 | 5.43584358e-10 |

CPU 检查对原始完整矩阵求 `numpy.linalg.eigvalsh(I)`，验证全部特征值正数，并检查有序主惯量 `λ1≤λ2≤λ3` 的 `λ1+λ2−λ3≥0`。仅检查三个对角元为正是不够的。

| link | 最小特征值 | 主惯量三角裕量 λ1+λ2−λ3 | 结果 |
|---|---|---|---|
| `base_link` | 0.00502085388 | 0.00451008159 | PASS |
| `link1` | 0.000716074888 | 0.000462788447 | PASS |
| `link2` | 0.000267820703 | 9.67351415e-5 | PASS |
| `link3` | 0.000580134382 | 0.000247829533 | PASS |
| `link4` | 0.000289626955 | 0.000176406535 | PASS |
| `link5` | 6.52695660e-5 | 6.10151567e-5 | PASS |
| `link6` | 2.75581662e-6 | 7.27388406e-7 | PASS |

7 个 link 的声明质量合计 `4.253378229099202 kg`。这些检查只排除明显无效的刚体参数；未验证材料、线缆、真实减速器、安装件、制造偏差或载荷。没有安装夹爪/吸盘，并不意味着真实末端质量可以从模型中随意删除。

## 4. Mesh、碰撞与末端参考点

7 个 visual 和 7 个 collision 的 origin 都是零、mesh 无显式 scale。URDF/SDF 当前碰撞均引用 `meshes/visual/*.STL`；归档另有 `meshes/collision/*.STL`。本次逐对 SHA-256/字节比较确认：**两个目录的同名文件完全相同**，改引用到 collision 目录并不会获得简化碰撞体（例如 `U:23–45`、`U:353–375`）。

| mesh（visual/collision 同名） | 三角面数 | 文件字节数 | link-local AABB 尺寸 m（显示舍入） | 对应文件同 hash |
|---|---|---|---|---|
| `base_link.STL` | 1216 | 60884 | `0.152 × 0.152 × 0.1155` | 是 |
| `link1.STL` | 3202 | 160184 | `0.163996 × 0.148 × 0.085` | 是 |
| `link2.STL` | 1634 | 81784 | `0.07 × 0.27 × 0.032` | 是 |
| `link3.STL` | 4678 | 233984 | `0.14 × 0.123973 × 0.066` | 是 |
| `link4.STL` | 5068 | 253484 | `0.054 × 0.085 × 0.165` | 是 |
| `link5.STL` | 3508 | 175484 | `0.066 × 0.054 × 0.051` | 是 |
| `link6.STL` | 2880 | 144084 | `0.044 × 0.0439256 × 0.025` | 是 |

所有 STL 满足二进制格式 `文件长度 = 84 + 50 × 三角面数`，顶点坐标均有限；单份 7 mesh 合计 22,186 个三角面。这不证明封闭流形、无退化面或完整实物外形。URDF/SDF/STL 的绝对尺度与真实尺量一致性仍 UNKNOWN，不能凭尺寸看起来合理便宣布校准成功。

`link6` mesh 在局部坐标中的范围约为 `x∈[-0.022,0.022]`、`y∈[-0.021962779,0.021962779]`、`z∈[-0.025,0]` m；可用于后续可视化核对，不能据此宣称 `link6` 原点就是厂商定义的法兰面中心或真实 TCP。原始模型没有 `flange/tool0/tcp` frame（`U:337–395`）。

最小参考点方案：离线和静态导入阶段把 `link6` frame 明确命名为“临时模型参考点”，使用 `T_link6,temporary = identity`。保留一个非物理 Xform/显式固定变换即可，不新增质量、惯量或碰撞体，也不引入一个可能被 importer 填充惯量的空 rigid body。报告与后续日志必须使用同一个点。

真实任务之前，需厂商法兰图与现场测量确认 `T_link6,flange`，再冻结 `T_flange,TCP`；模型内临时点可保留作比对，真实 TCP 另命名。当前不取 STL 边界、CoM 或无安装依据的长度作为真实工具偏置；圆心、半径、高度、周期均未锁定。

## 5. 尚未知的动态参数与仿真临时设置边界

| 参数 | 证据/现状 | canonical 与下一阶段处理 |
|---|---|---|
| 每轴最大速度 | U/S 都为 0 | UNKNOWN；原字段保留，不视作硬件限速 |
| 每轴加速度、减速度、jerk | U/S 无有效声明 | UNKNOWN；不猜数值 |
| 最大/连续/峰值力矩、温度降额 | effort 为 0，缺厂商额定表 | UNKNOWN；不从质量反推电机能力 |
| 转子惯量、armature、减速比、效率 | 模型未给出已验证值 | UNKNOWN；不得与刚体惯量混淆 |
| 摩擦、阻尼、回差、柔顺性 | U 未声明；S 有零 spring 参数 | UNKNOWN；零缺省不是测量结论 |
| 控制器 Kp/Kd、内部插值、servo dt、延迟 | 未由本任务硬件验证 | UNKNOWN；不能照搬 Factory/FR3 actuator |
| 安装、工作空间、实机软/硬限位、工具负载 | 缺标定/现场记录 | UNKNOWN；不得用于真实安全边界 |

**本次不需要为了静态导入填任何虚构动态上限。** 候选静态 USD 使用无 drive、零 stiffness/damping 且 timeline 不播放；`0` 在这里表示有意关闭仿真驱动，不是机器人动态参数。即使 importer 接受 `velocity/effort=0`，也不能认为动态模型已可用。

如果未来动态验证必须使用临时 velocity/effort/PD/armature，应在独立的仿真配置中逐项写明 `SIM_ONLY / uncalibrated / 来源 / 用途`，并与实机配置隔离；未核实前不输出可直接执行的实机参数。当前没有创建、安装或应用这些临时值。

## 6. 原始文件保护和适配变更方案

本次**未创建厂商 URDF 的修改版**；已用原 URDF 生成本地派生 USD，位于本项目已忽略的 `model_assets/local/b0_r1_usd/`。本次原有 `package://` URI 已解析为完整几何，无需改写 mesh 路径。未来需要适配时，仍仅写本项目私有目录，不写入 Factory、Isaac 安装目录或厂商源目录。

最小变更顺序：

1. 校验上表归档与模型 hash；在私有目录保留原始解压文件。
2. 生成独立适配 URDF，唯一必要的首项候选修改是把 `package://episode1_urdf_1113/...` 解析到私有解压目录中的真实 mesh 路径。解析器只接受此包名和归档内预期 mesh，禁止隐式下载；原 mesh 不修改。
3. 优先使用相对路径和原有包结构，检查导入器实际解析结果；若工具需要绝对路径，只在本地生成文件中写入并在变更记录中说明不可跨机器直接复用。自研脚本通过显式输入路径获取资源，不硬编码私人路径。
4. origin、轴、位置界、质量、CoM、惯量保持不变；暂不修改碰撞 mesh，也不把 SDF 的 0.1655 m 覆盖到 URDF。
5. 临时参考点使用非物理 Xform/明确固定变换，不伪造厂家法兰/TCP；若未来新增 fixed joint，单独记录其来源与合并策略。
6. 每次保存 `original hash / adapted hash / 修改字段 / 修改前后值 / 理由 / 测试结果`，CPU FK 验证适配前后不变；后续真实校准作为新版本、不得悄悄覆盖。

厂商 URDF、mesh、SDK、派生 USD、含厂商完整模型内容的 diff/归档始终留本地。提交范围仅限审查文档、自研通用脚本与不含厂商资产的合成测试；`.gitignore` 的 `model_assets/local/` 和 USD 规则只是第一层，提交前仍逐文件审查。

## 7. URDF → USD 最小静态导入方案

环境状态与启动证据见 [环境报告](B0_R1_ENVIRONMENT_AUDIT.md)。本节先列转换方案，随后单独记录已执行的静态 smoke 及其局限。

已只读核对本机 Isaac Lab 仓库：`v2.1.1 / 90b79bb2d44feb8d833f260f2bf37da3487180ba`；下文源码行号相对此提交。官方 [2.1.1 导入指南](https://isaac-sim.github.io/IsaacLab/v2.1.1/source/how-to/import_new_asset.html) 说明可以使用 `UrdfConverter` 与 headless 转换。以版本固定的[配置源码](https://github.com/isaac-sim/IsaacLab/blob/v2.1.1/source/isaaclab/isaaclab/sim/converters/urdf_converter_cfg.py)及[转换实现](https://github.com/isaac-sim/IsaacLab/blob/v2.1.1/source/isaaclab/isaaclab/sim/converters/urdf_converter.py)为 API 依据，不采用最新版 Isaac Sim 文档替代 4.5 的实现。

| 项目 | 最小设置/核验 | 来源与理由 |
|---|---|---|
| 输入/输出 | 私有原始 URDF → `model_assets/local/b0_r1_usd/` | 本次 URI 直接解析成功，无需适配；USD 含派生厂商几何，不提交 |
| `fix_base` | `True` | Lab cfg:90–91；converter:125–126；台式固定基座，不添加自由漂浮基座 |
| `root_link_name` | 显式 `base_link`；检查实际 `ArticulationRootAPI` 所在 prim | cfg:93–97；converter:79–81；不能假定外层 Xform 就是 articulation root |
| `merge_fixed_joints` | `False` | 当前无 fixed joint，保留 link/frame 便于比对；cfg:102–103 |
| 单位与坐标 | m/kg/rad；检查 `metersPerUnit`、Z-up 以及角度 API 单位 | converter:107–108 使用 distance scale=1；USD 的 angular joint 属性需注意度与 rad 转换 |
| drive | `target_type="none"`、stiffness=0、damping=0 | cfg:79–85；不用脚本默认的 position/100/1 |
| 惯量 | 导入原质量/CoM/六项 I；检查 USD `diagonalInertia + principalAxes` 重建原 I | 不拿 CAD mesh 自动密度重算替代已给 I；非对角项不可丢失 |
| mesh | 7 个 link visual、7 个 collision 引用均能解析；无隐式 scale | 缺一即失败，不能接受空几何 |
| collision | 静态初次 `convex_hull`，检查全部 collider；保留自碰撞核验要求 | cfg:117–127；hull 可能外扩；以后运动前需要碰撞对/几何审查，不能仅关闭自碰撞来消除错误 |
| 缓存 | 独立输出路径、`force_usd_conversion=True` | converter:28–31：仅 mesh 改动未必触发缓存更新 |
| timeline | 不播放、不创建策略、不设置 target、不调用 physics step | 只转换、打开并检查 USD 元数据与静态世界变换 |

本机 Sim 4.5 缓存扩展为 `isaacsim.asset.importer.urdf-2.3.10+106.4.0.lx64.r.cp310`，其 `docs/CHANGELOG.md:97` 记录“可用时始终导入惯量”。Lab converter 本身没有显式设置 inertia 开关，故实际输出仍需检查，不能把文档声明当作本模型导入结果。现有 `standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py:68–99` 会设轮速并播放 timeline；**不使用该运动示例**。

最小静态导入的验收条件：

1. 模型解析、7 份几何和输出 USD 成功，无 missing asset、非法 inertia、模型解析失败警告；保留 stdout、stderr 和 importer log。
2. 存在单一正确 articulation root、正确世界固定关系、7 个刚体、6 个 revolute DoF，零位/位置界与 URDF 对齐；固定基座辅助 joint 不计为第 7 个可动 DoF。
3. 显式按名字读取实际 joint 顺序，再建立 `joint1…joint6` 的映射；不假定字典、USD 遍历或 PhysX 顺序就是策略顺序。
4. 在 timeline 停止下检查 q=0 的 link 世界变换、参考点与 CPU FK；位置与姿态使用预先记录的数值容差比较，不能只看截图。USD angular 属性与四元数顺序须按 API 转换。
5. 从 USD MassAPI 的质心、质量、主惯量及主轴姿态重建完整惯量，与原 URDF 数值比较；检查是否有自动填值或意外重算。
6. 检查 collision proxy/convex hull 是否存在、是否过分膨胀、相邻碰撞排除是否符合预期；静态导入通过仍不代表自碰撞或桌面风险已通过。
7. 退出程序并确认没有 timeline play、trajectory、policy、ROS/CAN；记录命令、版本、输入 hash、输出 hash 和耗时。失败则停止，按日志归因，不通过升级环境或编造动态参数绕过。

可以复用 Lab 现成 converter；无需新建 RL 环境、actuator 抽象、任务注册或训练配置。静态导入之后，只有另行授权的 B1 才讨论 physics step、受控姿态或控制测试。

### 本次静态 smoke 实际结果：PARTIAL

执行入口为本仓库 [smoke_import_urdf.py](../../scripts/smoke_import_urdf.py)。复用环境报告中的 `FACTORY_ROOT` 和既有 launcher；以下变量只在当前进程生效：

```bash
env FACTORY_VENV="$FACTORY_ROOT/work/envs/factory-g01" \
  bash "$FACTORY_ROOT/outputs/isaac-factory-repro/scripts/run_sim.sh" \
  "$PWD/scripts/smoke_import_urdf.py" --headless --device cpu \
  --urdf "$PRIVATE_MODEL_DIR/episode1_urdf_1113.urdf" \
  --usd-dir "$PWD/model_assets/local/b0_r1_usd"
python3 -c 'import json; x=json.load(open("model_assets/local/b0_r1_usd/static_import_summary.json")); assert x["geometry_checks"] == "PASS" and x["physics_steps"] == 0 and not x["timeline_played"]'
```

不能只检查 shell 退出码：必须同时检查本次新写的 summary 和日志 `B0_R1_STATIC_IMPORT=` 标记。脚本运行前清除同名旧 summary，避免旧结果掩盖失败。本次最终进程退出 0，summary 总状态 `PARTIAL`、`geometry_checks=PASS`。经过审查的数值记录与派生 USD 的 SHA-256 见 [B0_R1_STATIC_IMPORT.json](data/B0_R1_STATIC_IMPORT.json)；原始日志在本任务 `work/r1_smoke/static_import_final.log`。

| 已执行检查 | 数值/实际结果 |
|---|---|
| 刚体 / revolute joint | 7 / 6；按名字检查 body0/body1 父子链和位置限位的 rad→deg 转换 |
| 世界固定与 articulation root | 唯一 `/episode1_urdf_1113/root_joint` 是固定 joint；body0 为空（world），body1 为 base_link |
| q=0 全 link FK | 与 CPU URDF 最大位置差 `4.322e-8 m`、姿态差 `1.725e-7 rad`；脚本预设纯数值容差 `1e-6 m / 1e-5 rad` |
| 关节轴 | 把 USD `localRot0` 和 axis 转回父/基坐标再比较；最大方向向量差 `1.385e-7`，未按 USD axis 枚举文本判断 |
| 质量 / CoM / 完整惯量 | 7 组通过；CoM 最大差 `5.719e-9 m`；`principalAxes × diagonalInertia` 重建 I 最大元素差 `4.018e-10 kg·m²` |
| visual / collision 几何 | 14 mesh，每个 link 的 visuals/collisions 都有非空有限顶点；需遍历 USD instance proxies |
| 单位 | metersPerUnit=1、Z-up |
| 驱动 / 时间线 | 所有 angular drive stiffness=damping=0；无 target 下发，timeline 未播放，physics step=0 |
| 资产依赖 | 无未解析模型/几何依赖；通用 `UsdUtils` 仍报告3个 `OmniPBR.mdl` 材质源未解析 |

`OmniPBR.mdl` 在现存 Sim 的 `kit/mdl/core/Base/OmniPBR.mdl` 实际存在；通用 USD 依赖扫描未验证 MDL 搜索路径或渲染效果，因此保留 **PARTIAL**，没有修改材质或全局搜索路径来掩盖警告。日志另有 `DriverShaderCacheManager::init() called without a shutdown()` 等非阻塞消息，也保留在运行证据中。未进行视口渲染、物理初始化、控制器或运动测试；Physics DoF 索引顺序仍须在后续授权步骤按名称确认。

失败也已保留：首版审计工具普通 `stage.Traverse()` 漏过 instance proxies，误报无 mesh；修正后确认14 mesh。严格要求零未解析项的重测因上述 MDL 警告失败，尝试 `fast_shutdown=False` 的该次失败运行在关闭后段错误（exit139）；恢复 Factory 既有默认快速关闭后最终运行正常返回且有完整 summary。它们是本次导入/审计与关闭路径的限制，不能据此宣布完整环境或渲染稳定。没有修改已安装环境或原始 URDF。

## 8. 进入 B1 前的资料与决策门

**B1 静态模型导入的输入条件已具备，并已在 B0-R1A 授权范围内完成静态转换检查；总体 PARTIAL。** URDF/SDF 尚未完全一致并不妨碍明确选定 URDF 做后续导入，但不得宣称两者可互换。材质解析、完整碰撞审查与运行时 PhysX DoF 验证尚未完成；B1 运动/动态验证仍未放行。

需要用户/厂商后续提供：准确型号与硬件版本；匹配版本的官方模型及 URDF/SDF 高度差说明；六轴正方向、机械零位与编码器零偏/单位/顺序/减速比的标定资料；真实软/硬位置界与速度、加速度、连续/峰值力矩规格；法兰机械图及安装朝向、桌面/底座尺寸、末端负载清单；控制器位置伺服模式、可更新目标语义、时延/状态时间戳/停止文档。仅收集资料不涉及本次连接硬件。

理解验收（待研究者回答，不标学习 PASS）：

1. 为什么不能把 `joint2/3/4` 的模型符号变换直接当作真实编码器映射？
2. 为什么非零质量、三个正对角惯量仍不足以证明惯量矩阵有效？
3. 为什么无驱动、timeline 停止的 USD 导入通过，不代表 PPO 或实机运动安全？

未检查：实机标定、动态辨识、渲染效果、碰撞净空和 B1 运动；未生成修改版 URDF。风险：候选 URDF 中的零动态界和未校准 TCP/控制动态仍阻止真实部署。
