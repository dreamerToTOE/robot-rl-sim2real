# B0 控制指令、反馈与在线控制矩阵

审查日期：2026-11-13（KST）
仓库 commit：`8f6ebe5d6ea4d958e635e50da1f0e05c7e452022`
判定规则：PASS 只表示源码/静态证据足够；PARTIAL 表示接口表面存在但关键行为未验证；UNKNOWN 表示二进制或硬件内部不可见；BLOCKED 表示当前不能安全用于实机 PPO。

## 1. 指令矩阵

| 接口 | 输入与单位 | 源码能确认的动作 | 频率/时序 | 未确认项 | B0 结论 |
|---|---|---|---|---|---|
| `angle_mode` action | `float64[6] angles`（度）、`speed_ratio`，`AngleMode.action:1-3` | 节点一次调用 `execute_motors_degrees_normal`；随后 10 段 sleep 反馈（`robot_interface_node.py:553-620`） | 单个 goal；无流式发送 | 未校验角度维度、NaN、URDF 限位或 speed_ratio；是否可覆盖未完成 goal、内部规划、实际完成判定 | BLOCKED 作为 PPO action；不适合作为高频流 |
| `FollowJointTrajectory` | 标准 `JointTrajectory`，位置/速度/`time_from_start`（`robot_interface_node.py:735-770`） | 非空校验；每段插入 15 个中间点；逐点调用 `dynamic_move_with_velocity`（`:773-884`） | 实际发送循环没有显式 sleep；不能把点数或规划时长当控制率 | `.so` 是否按时间调度、排队、覆盖、抢占、丢包处理 | BLOCKED 作为在线闭环；PARTIAL 作为低频实验候选 |
| `dynamic_move_with_velocity` | 二进制方法的 `trajectory_point`，docstring 提到 positions/velocities/time | 离线 fake-object 探针看到读取 `time_from_start`、内部等待和被拦截的 CAN 写调用 | 假对象下 1 s→约 1 s 等待；真实设备周期未知 | 是否覆盖/排队、速度单位/钳位、硬件接受语义 | PARTIAL（黑盒内部）/UNKNOWN（实机） |
| `linear_move` action | XYZ mm、欧拉角度、模式、步长 mm（`LinearMove.action:1-5`） | 单独线程执行 `ros2_linear_move_execute`（`robot_interface_node.py:438-500`） | 由 IK/内部步进决定；反馈约 0.1 s 一次（`:498-519`） | 取消不能完全停止，内部步进频率未知 | BLOCKED 作为 PPO 接口 |
| `move_xyz_rotation` action | XYZ mm、旋转角度、IK 模式、速度比例（`MoveXyzRotation.action:1-5`） | IK 后一次 normal angle motion（`robot_interface_node.py:631-733`） | 点到点；进度为模拟 sleep | IK 与实际状态闭环、轨迹覆盖、限速 | PARTIAL；不用于在线 PPO |
| 二进制 `position_mode` / `speed_mode` / `sync_motion` | Python 签名可见，内部参数语义不完整 | `.so` 导出符号存在 | 未测 | CAN 帧、持续流式能力、限制和停止 | UNKNOWN |
| 速度控制 | 仅二进制 `speed_mode(address,direction,speed,acceleration,sync_flag)` | ROS 节点没有速度 action/topic | 周期、单位、持续时间未知 | 是否能安全闭环、停止和限速 | UNKNOWN |
| 力矩控制 | 未发现 ROS 消息或 SDK 方法 | 无可用入口 | 无 | 扭矩反馈/命令均未知 | UNKNOWN / NOT PROVIDED |

**重要区分：** `FollowJointTrajectory` 发送完整目标列表；即使内部确实按时间播放，也仍是预先生成的轨迹播放，不能称为 PPO 根据每次实时观测更新动作的闭环。

## 2. 状态反馈矩阵

| 反馈 | 字段/单位 | 名义周期 | 时间戳和新鲜度 | 可直接构造的 observation | 结论 |
|---|---|---:|---|---|---|
| `joint_states` | 6 个 joint 名称，位置 rad；velocity/effort 为空（`robot_interface_node.py:270-309`） | 120 Hz 定时器（`:130-136`） | `header.stamp` 在发布时生成，不是设备采样时间 | `q` 可用；`dq` 只能由时间戳角度差分估计 | PARTIAL |
| `motor_angles` | 6 个原始电机角度（度）、ROS timestamp、success/message（`MotorAngles.msg:1-13`） | 30 Hz 定时器（`robot_interface_node.py:137-141`） | timestamp 在节点发布时生成；异常时发布缓存角度 | 原始角度可用于诊断；必须经过已验证 offset/sign/ratio | PARTIAL |
| `read_motor_angles` service | response 6 轴度数、success、message（`ReadMotorAngles.srv:1-6`） | 按请求 | 服务回调返回 `cached_motor_angles`（`robot_interface_node.py:311-344`） | 不能当作实时采样 | PARTIAL |
| action feedback | current_angles、progress、status | `angle_mode`/位姿为模拟进度；直线约 10 Hz | 反馈不等于独立关节测量 | 只能做 UI/诊断，不能替代状态 topic | PARTIAL |
| 力矩/电流 | 节点与消息没有字段 | 未知 | 无 | 不能输入 torque observation 或做负载保护 | UNKNOWN |

硬件核心的 `read_motor_angles(timeout=0.05)` 签名由 CPython wheel introspection 看到，但是否返回新采样、超时如何标志和真实周期均未验证。该默认超时接近 120 Hz 状态定时器的 8.33 ms 周期的 6 倍，存在回调阻塞和状态过期风险。

离线 fake-object 探针没有打开 PCAN 或电机：它只替换 `WriteMessages` 并检查调用次数/等待时间。因而不能把约 1 s 等待或约 12 次写调用当作真实控制频率，也不能把 velocities 的黑盒 payload 差异当作已验证的速度接口。

对空 positions 或 5 维 positions 的非法轨迹点，黑盒探针曾触发段错误（仍未接 CAN）。这使长度、顺序、NaN、单位、时间单调性和限位检查成为进入 SDK 前的硬门禁，而不是可选日志。

## 3. 时序与安全矩阵

| 层 | 当前证据 | 能否作为 PPO 频率 |
|---|---|---|
| physics/policy | 仓库仅有 `CircleXY` 数学生成器；没有 MDP、Actor 或训练配置 | NO |
| ROS timer | 30 Hz/120 Hz 名义值 | 只能作为发布请求率，不能作为端到端闭环率 |
| action callback | Python action server + `MultiThreadedExecutor(4)`（`robot_interface_node.py:920-927`） | 未测调度抖动，不能当硬实时 |
| SDK/CAN | `.so` 黑盒；无源码/实测日志 | UNKNOWN |
| state delivery | ROS topic/service；无过期门禁、无丢包窗口 | BLOCKED，需新鲜度协议 |
| stop/watchdog | `.so` 有 `emergency_stop`/`set_emergency_stop` 方法；离线探针只证明软件层遍历地址发帧，ROS 节点没有独立 stop service；直线取消无法完全停止；Reentrant + 4 线程无 active-goal 互斥 | BLOCKED |

代码中 FollowJointTrajectory 的真实 FPS 调试段被注释（`robot_interface_node.py:885-901`），所以没有实际交付频率证据。Actor forward 时间也不能替代 servo 或 state delivery 周期。

## 4. Observation/Action 适配判定

项目契约给出的候选 observation 为 `[q, dq, p_d-p_ee, v_d, a_prev]`，6 DoF 暂为 24 维（`docs/INTERFACE_CONTRACT.md:12-21`）。在当前证据下：

- `q`：可以从 `joint_states` 获得，但必须先冻结关节顺序、offset、符号和单位。
- `dq`：硬件消息没有反馈；只能用真实状态时间戳差分估计，并定义无效窗口。
- `p_ee`：需使用与实物法兰/TCP 一致的 FK；当前模型没有 TCP，且 URDF/SDF 轴冲突。
- `v_d`：`CircleXY.sample` 可提供参考速度（`src/robot_rl_sim2real/reference.py:30-38`）。
- `a_prev`：策略/安全包装器可记录，但当前尚未实现。

因此 observation 只能标记 **PARTIAL/待冻结**，不能把仿真 ground-truth 速度或力矩偷偷输入实机策略。

候选 action 应保持 `Actor [-1,1]^6 → 独立安全包装 → 有界关节位置参考`。在 SDK 证明“覆盖上一条目标、周期和停止”之前，`bounded_delta_joint_target` 只能作为设计候选，不能发送。

## 5. 下一步验证矩阵（不自动执行）

| 验证 | 前置审批 | 通过证据 |
|---|---|---|
| 二进制 API 的离线签名/错误路径 | 用户批准安装私有 wheel；不连 CAN | 版本/hash、导入日志、无硬件副作用 |
| ROS 状态新鲜度 | 只读状态权限 | 设备采样时间、发布/传输延迟、过期/丢包行为 |
| 目标覆盖与周期 | 现场安全审批、禁止策略 | 受控低速单轴/小位移日志，证明旧目标是否被替换 |
| stop/timeout | 急停在场 | 软件 stop、通信断开、超时和实体急停的独立证据 |
| shadow actor + 故障注入 | B5 门禁 | NaN、越界、过期、失联均拒绝，不产生硬件动作 |

本阶段不启动这些验证，不连接或使能真实机器人。

## 理解验收题

1. 为什么 action result 的 `success` 不能替代独立的关节状态反馈？
2. 为什么名义 120 Hz ROS timer 不能自动证明 120 Hz 真实伺服？
3. 为什么完整 `FollowJointTrajectory` 播放不是 PPO 在线闭环？
