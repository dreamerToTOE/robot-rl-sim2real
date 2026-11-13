# B0 空中圆形轨迹 Sim2Real 可行性

审查日期：2026-11-13（KST）
范围：固定平面、低速、小半径、无相机、无夹爪/吸盘、无接触。
结论：**理论上可行，当前实机闭环 BLOCKED；不得锁定圆心、半径、周期、姿态或 PPO 频率。**

## 1. 任务可行性判定

| 问题 | 当前证据 | 判定 |
|---|---|---|
| 六自由度是否能产生平面圆 | 静态 URDF 为 6 个 revolute joint，链路完整（`student_ros/src/episode1_urdf_1113/urdf/episode1_urdf_1113.urdf:87-394`） | PARTIAL：有运动学链，不等于实机可达 |
| 是否需要相机 | 位置由关节反馈 + FK 计算；项目契约明确不需要相机（`docs/INTERFACE_CONTRACT.md:12-21`） | PASS（架构层） |
| 是否需要固定姿态 | 为避免姿态漂移，首个任务应固定一个经审批的法兰/TCP 姿态；当前 TCP 未定义 | PARTIAL |
| 参考点 | URDF 只有 `link6`，无 `tool0/TCP`；需现场测量并在适配模型中固定 | BLOCKED |
| 圆心/半径/高度 | 必须从 FK、桌面/底座间隙、自碰撞和关节留量反推；当前没有安全几何或真实安装 | BLOCKED |
| PPO action | 设计合同优先 `bounded_delta_joint_target`；SDK 未证明目标覆盖/周期 | BLOCKED |
| 真实闭环 | 状态可名义发布 30/120 Hz，但无设备时间戳、dq、实际 servo/CAN 周期和独立安全层 | BLOCKED |

当前仓库 `CircleXY` 只做数学采样（`src/robot_rl_sim2real/reference.py:17-38`），4 个 CPU 单测通过；它没有 FK、IK、碰撞、控制或硬件连接。

## 2. 圆轨迹参数选取算法（先算法，暂不填实机数值）

1. **冻结坐标约定。** 用经核验的 `base_link` 作为参考坐标；测量真实安装姿态、桌面高度、底座包络、法兰/TCP 位姿。明确 URDF 还是 SDF 为 canonical，并修复 joint axis/origin 差异。
2. **选择安全初始关节配置。** 从真实反馈得到 `q0`，要求每轴离已验证软/硬限位有留量；不使用程序常量 `[180,90,83,30,110,30]` 作为安全事实。确认 joint order、offset、符号、减速比。
3. **用 FK 得到候选平面。** 对 `q0` 和候选固定姿态计算末端参考点 `p_ee`；候选 `z_c` 必须高于已审批桌面/障碍净空，且不能依赖相机补偿。
4. **从小到大搜索半径。** 令 `p_d(t)=[x_c+r cos(ωt), y_c+r sin(ωt), z_c]`。先使用极小 `r` 和低 `ω` 的离线候选，逐点求 IK/关节轨迹；只有在每个点存在连续解且关节留量、速度、加速度均满足厂商已验证界限时才扩大候选。
5. **做全周期几何检查。** 对整圈和 reset/停止过渡检查 self-collision、底座/桌面碰撞、法兰扫掠体积和奇异性（Jacobian 条件数等）。当前碰撞 mesh 使用 visual mesh，必须先验证其碰撞意义。
6. **做安全审批。** 由现场人员确认工作空间、急停、低速窗口、人工在场、通信超时策略；记录候选参数来源。B0 不锁定任何数值。

该算法输出“候选集合和拒绝原因”，不能直接输出实机目标。

## 3. 末端姿态和 FK 误差定义

- 参考点应是实际用于任务验收的法兰/TCP 点，而不是默认把 `link6` 原点当 TCP。必须在模型中增加独立适配 fixed joint 或显式变换，并保留厂商原始包不变。
- 离线 URDF FK 在 q=0 时给出 `link6` frame 约 `[0.302000,-0.000002,0.422001]` m；该数值只用于静态链路自检，不能直接作为真实圆心、高度或 TCP。
- 首个任务建议固定姿态，只跟踪位置误差 `e_p=p_d-p_ee`；是否允许绕工具轴旋转需在安全审查后决定。策略 observation 可带参考切向速度 `v_d`，避免同一点的时间歧义。
- 实机验收必须使用独立方法读取实际 `q` 并用同一 canonical FK 重建 `p_ee`，同时记录 `raw action → safety-clipped target → observed q → FK point`。Action 成功或仿真轨迹漂亮都不能替代这一证据。

## 4. Sim 与 Real 的最小可部署架构

```text
Isaac Lab canonical model
        ↓
PPO actor export ([-1,1]^6)
        ↓
Observation adapter (q, estimated dq, FK p_ee, reference, a_prev)
        ↓
Independent safety layer (finite/stale/limit/collision/timeout checks)
        ↓
Verified robot adapter (offset/sign/order/units, target overwrite semantics)
        ↓
Vendor ROS 2 / SDK / CAN
        ↑
Timestamped state feedback + independent FK reconstruction
```

### 可以直接复用

- `src/robot_rl_sim2real/reference.py` 的圆轨迹数学和 CPU 单测。
- `configs/task_template.yaml` 的字段骨架，但数值继续保持 `null`，直到 B0/B1 证据冻结。
- `docs/INTERFACE_CONTRACT.md`、`docs/SAFETY_GATE.md` 的日志字段和门禁思想。

### 必须重新实现或补齐

- canonical URDF/SDF 适配版、TCP fixed transform、FK/IK 和 mesh/collision 验证。
- 真实编码器到策略关节的 offset/sign/order/ratio 适配器，含单位和 stale-state 检查。
- observation adapter：无硬件 dq 时只使用带真实时间戳的估计，并拒绝无效窗口。
- 独立 action safety wrapper：有限值、软硬限位、速度/加速度/增量、超时、停止和可诊断拒绝原因。
- 策略导出、shadow actor、离线 replay、故障注入和动作审计日志。
- 验证后的 ROS/SDK 目标发送层；不能把现有 demo 或预生成轨迹播放当策略闭环。

### 当前接口不确定性

`dynamic_move_with_velocity`、`position_mode`、`speed_mode`、`sync_motion` 和停止 API 仅存在于 CPython `.so`，内部行为未知。`FollowJointTrajectory` 源码只证明“接收完整轨迹、插值、逐点调用”，没有证明目标覆盖、排队、控制周期或实时状态反馈。Isaac Lab 当前环境 import 失败，无法开始导入验证。

## 5. 主要阻碍和风险

1. **模型定义差异：** URDF 的 joint2/3/4 轴与 SDF 文本不同（joint6 相同），SDF 把各轴界限改为 `[-3,3]`。B0-R1A 更正：不能仅凭轴符号判断 FK 不等价，统一角度定义后的结果见 [数值比较](B0_R1_URDF_SDF_FK_COMPARISON.md)；限位也不等于全部放宽。
2. **动态限制缺失：** URDF/SDF 的 effort/velocity 全为 0，不能训练或下发时用作真实上限。
3. **TCP/安装未知：** 无 `tool0`/TCP，也没有桌面、底座和自碰撞审批，圆心/半径/高度不能填写。
4. **状态不足：** `JointState` 无 velocity/effort，服务返回缓存；只能在定义真实时间戳和估计窗口后使用 dq。
5. **控制时序未知：** ROS 30/120 Hz 是定时器名义值；离线 fake-object 探针只看到 `.so` 按 `time_from_start` 等待并产生被拦截的 CAN 写调用，真实 CAN 周期、队列、覆盖和停止仍未测量；`read_motor_angles` 的默认 50 ms 超时还可能阻塞 120 Hz 发布。
6. **安全缺口：** 节点没有独立 watchdog/紧急停止服务；FollowJointTrajectory 缺少维度、顺序、NaN、限位和单调时间校验，取消路径仍可能被外层报告为成功；非法维度轨迹点对二进制 fake 探针曾导致段错误；Reentrant + 4 线程没有 active-goal 互斥；直线动作取消明确不能完全停止。
7. **环境阻塞：** Isaac Sim 4.5 可 import，但 isaaclab 环境 0.54.3 指向缺失的 Isaac Sim 5.1，当前 `import isaaclab` 失败。
8. **交付入口风险：** 压缩包 `setup.py` 的 `moveit_bridge` entry point 指向未随包提供的 Python 文件，尚未完成 colcon 安装验证。

## 6. 放行建议与审批请求

本次不启动 PPO、不运行 Isaac、不连接或使能真实机械臂。进入 B1 前需研究者批准：

- 以私有包为输入的离线 XML/FK 审查和独立适配版生成；原始 hash、许可和 diff 留档。
- 选择并修复 Isaac Sim/Isaac Lab 配对环境；只做 CPU/短 smoke，不升级系统驱动。
- 在现场安全责任人批准后，单独验证只读状态、映射、目标覆盖、周期、stop/timeout、shadow 和故障注入；在这些证据齐全前只读。

在上述证据完成前，本任务的理论可行性不能升级为实机可行性，也不能宣布 Sim2Real 成功。

## 理解验收题

1. 为什么一个 10 ms 的 Actor forward 不代表 100 Hz 的真实闭环？
2. 为什么圆心、半径和周期必须由 FK/碰撞/限位审查产生，而不能从示例脚本直接拿来上机？
3. 为什么固定姿态仍然需要测量并冻结真实 TCP？
