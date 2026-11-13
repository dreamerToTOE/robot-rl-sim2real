# B0-R1A：URDF / SDF 数值 FK 一致性检查

审查日期：2026-11-13（Asia/Seoul）。状态：**PARTIAL**。

结论：两份文件不能在相同 `base_link`、相同数值关节角、相同位置限位下直接互换。统一模型关节角符号后，末端 FK 仅剩约 **0.5 mm 的基坐标 Z 向偏移**及约 `1.60e-11 rad` 的旋转数值残差；再作固定基坐标注册，307 个离线配置的末端 FK 在 `1e-8 m / 1e-8 rad` 数值比较容差内一致。但该注册使 `base_link` / `link1` 错开 0.5 mm，**不能证明整条链、安装基准或真实机器人相同**。位置限位仍不一致。

## 输入、执行和证据

本机私有 `student_ros.zip` 的 SHA-256：

`bbc42d6efb76aeb7eea48897d231b5bc3a20d0e081f08816722cd048872d5d4e`。

以下路径均相对私有解压目录；原文件只读，未创建修复模型、未提交厂商文件：

| 文件 | SHA-256 |
|---|---|
| `student_ros/src/episode1_urdf_1113/urdf/episode1_urdf_1113.urdf` | `78c1b35c65fe747876c19c06c3ca3492d22aeac4416c536f585da494ef6842ae` |
| `student_ros/src/episode1_urdf_1113/urdf/robot.sdf`（SDF 1.11） | `352f42d261cb4e17590dae1e36a7d06c5db19978b85ed757aefdff92a69cde26` |

复现命令（`PRIVATE_MODEL_DIR` 指向上述 `urdf` 目录）：

```bash
python3 scripts/compare_model_fk.py \
  --urdf "$PRIVATE_MODEL_DIR/episode1_urdf_1113.urdf" \
  --sdf "$PRIVATE_MODEL_DIR/robot.sdf" \
  --tip link6 --seed 20261113 --random-count 256 \
  --output docs/reports/data/B0_R1_FK_COMPARISON.json
python3 -m unittest discover -s tests -v
```

脚本只用 Python 标准库；本次宿主 `python3` 执行成功。原始数值、输入 hash、脚本 hash、每配置误差、限位越界标记、逐 link 残差与方向差异保存在 [B0_R1_FK_COMPARISON.json](data/B0_R1_FK_COMPARISON.json)，`representative_poses` 保存每类首个样本双方完整末端变换。没有厂商 XML、mesh、SDK 或二进制内容。公开单元测试模型均是独立构造的二轴模型，与厂商资产无关。

## 解析语义和完整链

两份文件均为 `base_link → joint1 → link1 → joint2 → link2 → joint3 → link3 → joint4 → link4 → joint5 → link5 → joint6 → link6`，7 links、6 revolute，无固定关节。名称、顺序和各 joint 的 parent/child 完全相同（URDF 六 joint 段 `:87–104 / :145–162 / :203–220 / :261–278 / :319–336 / :377–394`，SDF `:39–56 / :94–111 / :149–166 / :204–221 / :259–276 / :314–331`）。

SDF 的 `pose relative_to` 定义初始位姿参照；joint 省略 pose 参照时以 child link 为默认，axis 的 `expressed_in` 未指定时以初始 joint frame 表达。依据：[SDFormat 官方 frame 语义](https://sdformat.org/tutorials/specification/pose_frame_semantics/1.7/)。脚本先解析初始 frame 图，再求 `T_parent_joint` 与 `T_joint_child`，使用 `T_parent_joint R(axis,q) T_joint_child` 递推，避免将 SDF joint pose 当成 URDF origin 文本照搬（`scripts/compare_model_fk.py:127–134,166–240`）。

脚本支持本模型所用的单一平面层级 SDF 1.7–1.11、本地 frame、Euler/rad、fixed/revolute；未知 frame、循环、重复名、闭链、nested/include/placement-frame、外部 scoped frame、mimic、旧 axis frame 语义、四元数或角度制 pose 等明确失败。它不是 libsdformat 全实现。不支持的输入不能产生“PASS”（同文件 `:147–240`；合成拒绝用例 `tests/test_model_fk.py:125–146`）。

## joint origin、axis 和 limit 的数值比较

将 RPY 转为旋转矩阵后再比较，不能直接比较欧拉角字符串：

| 项目 | 结果 | 判断与证据 |
|---|---|---|
| parent / child | 六轴均相同 | PASS；见上述 joint 段与 JSON `joint_comparison` |
| joint2 origin | URDF Z=`0.166 m`，SDF Z=`0.1655 m`；其他 xyz / rotation 相同 | PARTIAL；真实局部几何差异 `0.0005 m`，URDF `:149`、SDF `:95` |
| joint3 RPY | URDF yaw=`3.1416`，SDF yaw=`-3.1415853071795872` | PASS（数值）；相差 2π，矩阵旋转残差 `6.43e-16 rad`，URDF `:208`、SDF `:150` |
| joint5 RPY | 三个欧拉角写法明显不同 | PASS（数值）；同一旋转的另一种表示，矩阵残差约 `1.60e-11 rad`，URDF `:324`、SDF `:260` |
| 其余 joint origin | 旋转残差≤`5.56e-17 rad`，位置残差仅浮点舍入 | PASS（数值）；JSON `joint_comparison` |
| axis（joint frame） | joint2/3/4：URDF `-Z`、SDF `+Z`；joint1/6 均 `+Z`，joint5 均 `-Z` | PARTIAL；可通过模型角符号映射统一，不能推断电机方向 |
| velocity / effort | 两份文件六轴全部写 `0 / 0` | UNKNOWN（真实动态能力）；本脚本保留原值，不补值、不用它定义可执行动作 |

穷举 `2^6=64` 个符号组合，在 q=0 及各轴单独小角探针上比较所有 link 的位置与旋转误差，最佳组合是：

```text
q_sdf = diag(+1, -1, -1, -1, +1, +1) q_urdf
```

最佳比较分数 `8.750000006e-6`，次佳 `0.01097497713`（位置平方与角误差平方之和，仅用作符号选择，没有物理精度含义；脚本 `:282–306`）。该映射**只描述两个模型**，本检查不需要额外角度零偏；真实编码器→URDF 的顺序、符号、offset 仍为 UNKNOWN。

| joint | URDF 原位置限位 rad | SDF 映射至 URDF 角定义后的限位 rad | 本次共同合法采样范围 rad |
|---|---|---|---|
| 1 | `[-3.141592654, 2.792526803]` | `[-3, 3]` | `[-3, 2.792526803]` |
| 2 | `[-1.570796327, 1.570796327]` | `[-3, 3]` | `[-1.570796327, 1.570796327]` |
| 3 | `[-1.448623279, 1.396263402]` | `[-3, 3]` | `[-1.448623279, 1.396263402]` |
| 4 | `[-0.523598776, 5.323254219]` | `[-3, 3]` | `[-0.523598776, 3]` |
| 5 | `[-1.919862177, 1.919862177]` | `[-3, 3]` | `[-1.919862177, 1.919862177]` |
| 6 | `[-0.523598776, 5.323254219]` | `[-3, 3]` | `[-0.523598776, 3]` |

这些是文件声明范围，尚不是现场批准的安全范围。SDF 不是“所有关节范围都更宽”：joint1 部分、joint4/6 的正向高角区域反而更窄；本次不覆盖或修改任何限位。

## FK 测试设计和结果

参考点选为两模型已有的 **`link6` frame 原点**，比较完整姿态；不是已校准法兰中心或真实 TCP。所有坐标均为各自 `base_link`，矩阵按 `Rz(yaw) Ry(pitch) Rx(roll)` 构造。

共同合法域内 283 个配置：q=0 共1个、每轴 `±0.01 rad` 共12个、固定 seed 的均匀随机256个、每轴近上下边界12个、所有轴同时近上下界2个。近界余量为该文件/共同区间宽度的 `1e-6`，仅为离线数值取样。另测 **URDF / SDF 各自原始限位附近各12个**，合计307配置；它们在另一模型越界时明确标记，不算共同可执行配置（脚本 `:264–279,340–354`）。

q=0，末端位置（m）：

```text
URDF: [0.3019995885966, -0.0000018476264, 0.4220011827655]
SDF : [0.3019995885966, -0.0000018476264, 0.4215011827646]
```

位置误差为同参考系的欧氏距离，姿态误差为相对旋转的最小角；没有使用易受 RPY 分支影响的逐角相减。以下先应用符号映射，再比较：

| 配置组 | 数量 | 同 base 最大末端位置差 m | 同 base 最大姿态差 rad | 固定基注册后最大位置差 m | 固定基注册后最大姿态差 rad |
|---|---:|---:|---:|---:|---:|
| q=0 | 1 | `5.000000009e-4` | `1.600e-11` | `5.552e-17` | `8.471e-22` |
| 单轴±扰动 | 12 | `5.000000009e-4` | `1.600e-11` | `5.129e-14` | `1.600e-13` |
| 随机合法配置 | 256 | `5.000000009e-4` | `1.600e-11` | `1.218e-11` | `3.188e-11` |
| 单轴近共同边界 | 12 | `5.000000009e-4` | `1.600e-11` | `9.565e-12` | `3.192e-11` |
| 全轴近共同边界 | 2 | `5.000000002e-4` | `1.600e-11` | `7.570e-12` | `3.078e-11` |
| 两模型各自原生近边界 | 24 | `5.000000009e-4` | 约`1.600e-11` | `1.021e-11` | `3.200e-11` |

若直接给两模型相同数值 q，不转换 joint2/3/4，随机组末端位置差最大 `0.728675 m`、姿态差最大 `3.140722 rad`；这说明执行或评估时遗漏角度定义转换会造成严重结果，但不说明机器人实际会产生该运动。

原生限位测试明确暴露的 URDF→SDF 越界例：joint1=`-3.141586720`、joint4=`5.323248372`、joint6=`5.323248372 rad`，均在 URDF 声明范围内、映射后不在 SDF 范围内。反向 SDF→URDF 也有 joint1 正向、joint2/3 双向、joint4 的 SDF 正向（对应 URDF 负向）、joint5 双向、joint6 负向越界，共9个；详见 JSON `source_boundary_samples`。**能数学计算 FK，不等于能够合法运动。**

## 固定坐标注册能解决什么

以 q=0 计算 `C = T_urdf(0) inverse(T_sdf(0))`，随后所有配置都只用同一个 `C` 左乘 SDF 末端 FK，未逐样本拟合。得到：

```text
C.translation ≈ [0, 0, +0.0005] m
C.rotation ≈ I（旋转角约 1.60e-11 rad，欧拉表示舍入）
T_urdf(q) ≈ C T_sdf(S q)
```

**末端函数在允许重新注册基坐标时数值一致：PASS。相同 base / 全链 / 位置限位一致性：PARTIAL。**

原因是 joint1 围绕 Z 轴转动，joint2 局部 origin 的 `+0.5 mm Z` 差异经过 joint1 后仍是固定的基坐标 Z 平移，对下游全部 link 保持相同偏移。符号映射和其余旋转等价解释了其他差异，无需臆造真实电机方向。

| link | 同 base、符号映射后最大位置差 | 对整个 SDF 应用同一个 C 后最大位置差 |
|---|---:|---:|
| base_link / link1 | `0 m` | 约`0.0005 m` |
| link2 / link3 / link4 | 约`0.0005 m` | ≤`9.615e-12 m` |
| link5 / link6 | 约`0.0005 m` | ≤`1.218e-11 m` |

上表来自共同域全部283配置，逐 link 姿态残差也保存在 JSON `all_link_comparison`。当 base_link 被规定必须相同时，整体坐标变换应为 I，而 link2 起仍有0.5 mm差异，因此不能把 C 偷放进真实 base/TCP 标定以宣布完整模型一致。joint2 高度应为哪个值，需厂商尺寸图或独立实测决定；本阶段保留 URDF 原值作为候选 canonical 基础。

## 运动方向检查

在 q=0 和前8个随机共同合法配置，用中心差分 `h=1e-6 rad` 分别比较线速度 Jacobian 列及空间角速度列（共9姿态×6轴；脚本 `:355–379`）。应用模型符号映射后，六轴最大线列差≤`8.34e-11 m/rad`、最大角列差≤`1.32e-10`；非零列夹角余弦均约1，方向一致。未映射的 joint2/3/4 正方向与另一模型相反。

joint6 围绕所选 `link6` 原点旋转，该原点线速度列为0、线方向余弦记录为 `null`，但角速度列非零并一致。因此不能仅看末端原点位置判断该关节方向；joint4 在零位的平移贡献也很小，需结合姿态和非零配置。

这些是文件运动学方向，**实际电机、SDK、编码器的正负方向尚未测试：UNKNOWN**。

## 验证边界与下一门禁

本次 CPU 单元测试12/12通过，其中新增8个合成 FK 测试方法：解析默认 child-frame、相对 frame、axis expressed_in、解析解、符号映射、原生/共同限位、不同欧拉表示、假基注册无法掩盖全链差异、几何变化不能被 q0 注册掩盖及不支持语义拒绝；其余4个是既有参考圆测试。未安装新依赖。

允许继续准备 URDF-based 候选模型的静态导入；本报告不单独放行 B1。真实 joint2 高度、安装 base、法兰/TCP、编码器零位和位置限位应由厂商资料/现场核验。环境与导入前提见 [环境报告](B0_R1_ENVIRONMENT_AUDIT.md)，模型冻结及 UNKNOWN 动力学项见 [canonical 方案](B0_R1_CANONICAL_MODEL_PLAN.md)。

未执行真实机器人、SDK、训练、碰撞或动力学仿真；数值容差不是机械臂精度、重复定位精度或安全距离，无法据此确定圆轨迹实机参数。
