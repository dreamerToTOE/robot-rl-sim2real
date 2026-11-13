# B0 模型、环境与 SDK 审查

审查日期：2026-11-13（KST）
仓库：`robot-rl-sim2real`，`main`，commit `8f6ebe5d6ea4d958e635e50da1f0e05c7e452022`
结论：**B0 BLOCKED；静态模型和接口证据已收集，但不能放行实机或 PPO。**

## 1. 证据边界

本次只读检查了当前仓库和本机找到的厂商包。厂商包是本机私有文件 `student_ros.zip`，SHA-256 为
`bbc42d6efb76aeb7eea48897d231b5bc3a20d0e081f08816722cd048872d5d4e`；只在 `/tmp` 临时目录解压，未复制进仓库。
报告中的路径均为压缩包内相对路径，例如 `student_ros/src/...`。仓库没有提交 URDF、mesh 或 SDK。

复核命令：

```bash
git fetch --prune origin && git pull --ff-only origin main
find /home/ubuntu2004 -type f -iname 'student_ros.zip'
sha256sum <私有路径>/student_ros.zip
unzip -t <私有路径>/student_ros.zip
python3 -m unittest discover -s tests -v
```

压缩包完整性为 `No errors detected`；仓库 CPU 测试 4/4 通过。后者只验证数学圆轨迹，不能证明 ROS、仿真或硬件。

## 2. 当前开发环境

| 项目 | 实测证据 | 判定 | 影响 |
|---|---|---|---|
| Ubuntu / kernel | `/etc/os-release`：Ubuntu 22.04.5 LTS；`uname -r`：6.8.0-138-generic | PASS | 可作为宿主，但不能代替厂商支持矩阵 |
| 系统 Python | `/usr/bin/python3`，3.10.12，无 virtualenv；系统 `import torch` 为 `ModuleNotFoundError` | PARTIAL | 不能用系统 Python 直接跑 PPO |
| Conda | `conda 24.3.0`；`base`、`isaac_env`、`isaaclab`、`unitree_rl_mjlab` 均存在，当前 shell 未激活 | PASS | 应先明确唯一受支持环境 |
| PyTorch | base `2.7.0+cu128`；isaac_env `2.11.0+cu130`；isaaclab `2.7.0+cu128`；unitree `2.13.0+cu130`；各自 `cuda.is_available=True` | PARTIAL | 版本分叉，不能把任一环境当项目冻结环境 |
| GPU / 驱动 | RTX 3070，驱动 580.178.04，8,192 MiB，总空闲约 5,917 MiB（检测时） | PASS | 仅说明资源存在；本次未启动训练 |
| CUDA 工具链 | `nvcc` 不存在；驱动报告 CUDA 13.0，PyTorch wheel 自带 cu128/cu130 | PARTIAL | 运行时可用不等于 Isaac 编译链已验证 |
| ROS 2 | `/opt/ros/humble`，`ROS_DISTRO=humble`，`ros2cli/rclpy` 可定位，daemon running | PARTIAL | 厂商 README 声明 Ubuntu 24.04 + Jazzy（`student_ros/README.md:3-7`），与当前 Humble 不同；未编译厂商包 |
| Isaac Sim | `isaac_env` 可 import `isaacsim`；独立安装版本 `4.5.0-rc.36+release.19112.f59b3005.gl` | PARTIAL | 只做 import，未启动 Kit/场景 |
| Isaac Lab | `isaaclab` 环境有 0.54.3 元数据，但 editable 路径指向不存在的 Isaac Sim 5.1，`import isaaclab` 实测失败 | BLOCKED | 不得开始 Isaac Lab 导入或训练 |

没有升级 CUDA、驱动、ROS、Isaac Sim/Lab，也没有执行 GPU 长任务。

## 3. URDF 静态审查

### 3.1 链、轴和关节范围

`student_ros/src/episode1_urdf_1113/urdf/episode1_urdf_1113.urdf` XML 解析成功，包含 7 个 link（`base_link`、`link1`…`link6`）和 6 个 revolute joint。父子链和 joint-local 轴如下：

| joint | parent → child | URDF axis | lower / upper (rad) | effort / velocity | 判定 |
|---|---|---:|---:|---:|---|
| joint1 | base_link → link1 | `+Z` | -3.141592654 / 2.792526803 | 0 / 0 | PARTIAL |
| joint2 | link1 → link2 | `-Z` | -1.570796327 / 1.570796327 | 0 / 0 | PARTIAL |
| joint3 | link2 → link3 | `-Z` | -1.448623279 / 1.396263402 | 0 / 0 | PARTIAL |
| joint4 | link3 → link4 | `-Z` | -0.523598776 / 5.323254219 | 0 / 0 | PARTIAL |
| joint5 | link4 → link5 | `-Z` | -1.919862177 / 1.919862177 | 0 / 0 | PARTIAL |
| joint6 | link5 → link6 | `+Z` | -0.523598776 / 5.323254219 | 0 / 0 | PARTIAL |

证据在 URDF 的 joint 段：`...urdf:87-104`、`:145-162`、`:203-220`、`:261-278`、`:319-336`、`:377-394`。

位置上下界可作为模型声明读取；**所有 `effort="0"` 和 `velocity="0"` 都按缺失/无效处理，不能当作真实力矩或速度上限**。真实上限必须来自厂商资料和受控验证，不能在 B0 填值。

### 3.2 质量、惯量、visual/collision 和 mesh

- 7 个 link 都有 `inertial/mass` 和完整六项惯量；质量（kg）依次为 `1.944345`、`0.623226`、`0.569589`、`0.549826`、`0.396344`、`0.151750`、`0.018297`。这是 URDF 静态字段，不是称重或辨识结果（`...urdf:7-86`、`:105-202`、`:221-318`、`:337-376`）。
- 7 个 link 都有 visual 和 collision。两者均引用 `package://episode1_urdf_1113/meshes/visual/*.STL`（例如 `...urdf:23-45`）；压缩包同时包含 7 个 `meshes/collision/*.STL`，但 URDF 没有引用它们。碰撞几何与可视几何复用的合理性和性能尚未验证，判定 **PARTIAL**。
- 归档内 visual/collision STL 均存在，`unzip -t` 通过，路径完整性判定 **PASS（归档层面）**；ROS package 安装后 URI 解析未测试。
- URDF 没有 `transmission` 或 Gazebo actuator/plugin 段；SDF 也没有用于真实控制的插件/传动声明。它们是几何/运动学输入，不是可直接运行的 Isaac Lab actuator 配置，判定 **PARTIAL**。

### 3.3 基座、末端和零位

- 根 link 是 `base_link`，链末为 `link6`；没有 `world`、`tool0`、`tcp` 或固定末端 joint。末端参考点尚未定义，判定 **BLOCKED**：FK 圆轨迹必须先测量法兰/TCP，并在适配模型中显式固定。
- 控制节点在回零后使用 `[180, 90, 83, 30, 110, 30]` 度作为默认目标（`student_ros/src/episode_controller/episode_controller/robot_interface_node.py:197-203`）。示例也随机扰动同一组角度（`student_ros/src/episode_controller/demo/client_demo.py:383-399`）。这只是程序常量，不是实机安全初姿态证据，判定 **UNKNOWN**。
- `joint_states` 发布时减去同一组 offset `[180,90,83,30,110,30]` 并转弧度（`robot_interface_node.py:291-296`），但没有符号翻转或显式排列转换。初始化还使用二进制对象的 `motor_ratios`（`:161-169`）。编码器→URDF 的 offset、符号、顺序和减速比没有实机核验，判定 **BLOCKED**。
- 同一离线 fake 探针在不连硬件时观察到：每个轴的 `q=+0.1 rad` 都使对应电机角增加约 `5.7296°`，没有看到 j2–j5 的负号；这支持“offset + rad→deg”这一黑盒路径，但不能证明编码器符号、URDF joint-local 轴或真实电机方向一致，仍需实机只读/小范围核验。
- 纯 CPU FK 复核（仅使用 URDF 的 `link6` frame、q=0，不代表真实 TCP）得到约 `[0.302000, -0.000002, 0.422001]` m；这只是检查链路数量和变换的静态证据，不能用作实机圆心或末端参考点。

## 4. URDF 与 SDF 差异

> B0-R1A 更正：joint6 的轴同为 +Z；axis 的文本差异本身不能证明 FK 不等价。统一关节定义后的数值结论见 [B0-R1 FK 比较](B0_R1_URDF_SDF_FK_COMPARISON.md)。本节保留 B0 静态字段记录。

`student_ros/src/episode1_urdf_1113/urdf/robot.sdf` 也能解析出 7 links/6 joints，但不能直接视为 URDF 的等价副本：

| 项目 | URDF | SDF | 风险 |
|---|---|---|---|
| joint1 limits | -π / 2.7925 | -3 / 3 | 数值不同 |
| joint2/3/4 axis | `-Z,-Z,-Z` | `+Z,+Z,+Z` | 需先统一 joint 角度定义再比较 FK；joint6 无轴差异 |
| joint2 origin z | 0.166 | 0.1655 | 0.5 mm 差异 |
| joint3 origin rpy | `0,0,3.1416` | `0,0,-3.141585...` | 约 π 表示不同，需验证转换语义 |
| joint4/5/6 pose | URDF origin/rpy | SDF 转换后的 pose | 不能仅凭文本相等判定 |
| limits | 每轴不同的 URDF位置界 | 所有轴约 `[-3,3]` | SDF 改变了边界，部分放宽、部分收紧 |
| effort/velocity | 全为 0 | 全为 0 | 都不能用于真实动态约束 |

证据：SDF joint 段 `robot.sdf:39-56`、`:94-111`、`:149-166`、`:204-221`、`:259-276`、`:314-331`。在 B1 之前必须选定唯一仿真 canonical 模型，逐关节对齐 axis、origin、limit、mesh 和 TCP；当前导入判定 **BLOCKED**。

配套 Gazebo launch 只启动 `robot_state_publisher`、`joint_state_publisher`、`ros_gz_bridge` 和 spawn（`student_ros/src/episode1_urdf_1113/launch/gz_simulator_launch.py:27-107`），SDF 没有关节控制插件/传动接口。因此“能可视化/生成模型”不等于“已有可控仿真执行器”；Isaac Lab 仍需独立 actuator、限位和时间步验证。

## 5. ROS 2 节点与 SDK 能力

### 5.1 源码可确认的 ROS 接口

- 节点注册 `FollowJointTrajectory` action：`episode_arm_controller/follow_joint_trajectory`（`robot_interface_node.py:54-60`）；另有 `angle_mode`、`move_xyz_rotation`、`linear_move` action（`:63-85`）。
- `AngleMode.action` 的目标是 6 个**度**和 `speed_ratio`，反馈/结果带当前角度和 `success`（`robot_arm_interfaces/action/AngleMode.action:1-15`）。回调调用一次 `execute_motors_degrees_normal`，然后以 10 段 `sleep` 模拟进度（`robot_interface_node.py:553-620`）。这不是连续目标流，也没有读取实际关节确认完成。
- `FollowJointTrajectory` 回调只检查点列表非空（`:735-770`）。执行端固定插值因子 15（`:848-860`），逐点调用 `self.motor_control.dynamic_move_with_velocity(trajectory_point=point)`（`:875-884`），源码没有显式按 `time_from_start` sleep；离线黑盒探针显示 `.so` 内部会等待时间戳，但真实设备上的覆盖、排队、周期和失败语义仍未知，判定 **PARTIAL/UNKNOWN**。
- 该回调没有校验 `joint_names` 顺序/数量、位置和速度维度、NaN、关节限位或 `time_from_start` 单调性（`:746-759`、`:801-841`）。取消时内层返回（`:875-880`），外层随后仍无条件 `succeed()` 并返回成功（`:761-770`）；反馈的 `actual.positions` 只是回显命令点（`:903-908`），不是实测关节角。作为安全接口判定 **BLOCKED**。
- `LinearMove` 使用 mm 和欧拉角，另起线程调用 `ros2_linear_move_execute`（`:438-500`）。取消分支明确写出“运动已开始，无法完全停止”，并 `join` 等待线程（`:501-509`），因此不能当作 PPO 的快速安全停止接口。
- `read_motor_angles` 服务返回 `cached_motor_angles`，不是在服务回调中重新读取 CAN（`:311-344`）。`motor_angles` 话题定时器名义 30 Hz（`:130-141`），`joint_states` 名义 120 Hz（`:130-136`）。`JointState.velocity` 和 `.effort` 始终为空（`:287-299`）。
- `MotorAngles` 消息含角度、ROS 时间戳和 success（`robot_arm_interfaces/msg/MotorAngles.msg:1-13`），但时间戳由节点在发布时生成（`robot_interface_node.py:233-236`），不是设备采样时间。实际发布、传输和调度抖动没有测量。

### 5.2 二进制 `.so` 能确认与不能确认的内容

CPython 3.10 x86_64 wheel `episode_controller_core-1.0.0` 只提供 `controller_core.cpython-310-...so`、PCANBasic、IK wrapper 和 `libik_wrapper.so`，没有 Python 源码。只读 import/`inspect.signature` 可看到以下公开方法：

`dynamic_move_with_velocity(trajectory_point=None)`、`read_motor_angles(timeout=0.05)`、`position_mode(...)`、`speed_mode(...)`、`sync_motion(...)`、`emergency_stop(sync_flag=0)`、`set_emergency_stop(active=True)`、`enable_motors(...)`。

这些签名和 docstring 只能证明 API 表面存在；以下均为 **UNKNOWN**：

1. `dynamic_move_with_velocity` 是否每次覆盖上一条目标、内部插值/速度规划、实际 CAN 发送周期和失败语义。
2. `angle_mode`/`execute_motors_degrees_normal` 是否阻塞到完成、是否排队、速度比例如何映射到真实限速。
3. `read_motor_angles` 的设备采样周期、返回是否缓存、超时后的数据有效性。
4. `emergency_stop` 是否真正切断运动、解除条件和硬件急停优先级。
5. `position_mode`、`speed_mode`、`sync_motion` 是否适合外部连续流式控制。

节点只在启动时构造 `MotorControl(usb_index)`（`robot_interface_node.py:205-219`），并通过 PCAN/CAN 访问；本次未安装 wheel、未打开 CAN、未调用构造函数，未做真实硬件验证。

节点构造函数默认 `usb_index=1`、`init_mode=1`（`:40-44`），启动就初始化 PCAN 并读取当前角度；`init_mode=0` 会逐轴回零并执行默认目标运动（`:172-203`）。因此不能把“启动 ROS 节点”当作无副作用的只读测试。

为减少误读，另做了一个**离线 fake-object 探针**：不构造 PCAN、不连 USB，只拦截二进制对象的 `WriteMessages`。探针观察到 `dynamic_move_with_velocity` 访问 `trajectory_point.time_from_start`、设置内部 `_trajectory_start_time`；时间戳为 1 s 和 2 s 的连续假点各等待约 1 s，每点约产生 12 次被拦截的写调用。固定 q/t 时把 velocities 设为空、0、0.2、2.0 rad/s 的 payload 相同，20 rad/s 才变化。这个证据只能说明黑盒中存在时间等待和脉冲/CAN 计算路径，不能证明真实电机接受、队列/覆盖、速度单位或安全限速，故内部时序为 **PARTIAL**、硬件行为仍 **UNKNOWN**。探针也显示析构会调用 PCAN `Uninitialize`；没有硬件句柄。

同一类无硬件探针把 `positions` 设为空或 5 维等非法长度时曾触发进程段错误；这不是实机测试，也没有接 CAN，但足以说明 ROS 层“只检查 points 非空”不构成输入安全边界。适配器必须在进入 `.so` 前严格检查 6 轴长度、顺序、有限值、单位、时间单调性和已验证限位。

二进制没有公开源码；`co_firstlineno` 只映射到 Cython 生成的 `controller_core.py` 行号，不能当厂商源代码行号。wheel `METADATA` 没有独立 LICENSE/COPYING/NOTICE；ROS package 的 `Apache-2.0` 声明（`episode_controller/package.xml:3-9`）不足以证明 mesh、IK 和二进制可再分发，许可判定 **UNKNOWN**。

另外，`episode_controller/setup.py:24-29` 的 entry point 引用 `episode_controller.moveit_trajectory_bridge:main`，而压缩包清单没有该文件；构建或运行该 entry point 可能失败，需在安装前修复或移除，判定 **PARTIAL/BLOCKED（未执行 colcon）**。

### 5.3 初步控制结论

| 候选 | 源码/文档证据 | B0 判定 |
|---|---|---|
| `bounded_delta_joint_target` | 项目接口契约要求的优先候选；厂商包没有对应 ROS API | UNKNOWN，需先做只读状态和单轴受控覆盖测试 |
| 低频分段 `FollowJointTrajectory` | ROS action 和 15 倍插值可见；执行时序/覆盖由 `.so` 决定 | PARTIAL；可作为实验候选，不能称为在线 PPO 闭环 |
| 一次性 `angle_mode` | 6 轴度数目标，调用一次 normal motion | PARTIAL；适合点到点，未证明可在未完成时更新 |
| 预生成整条轨迹播放 | `FollowJointTrajectory` 接收完整点列表 | BLOCKED 作为 PPO 在线闭环定义；没有每步新观测驱动 |

## 6. 安全与放行门

目前未发现 ROS 层独立 watchdog、过期状态拒绝、NaN/越界动作拒绝或紧急停止服务；只有二进制方法和 `set_free_mode` 服务。离线 fake 探针可看到 `emergency_stop`/`set_emergency_stop(True)` 会遍历 6 个地址发出软件层停止帧，解除只清内部标志；停止生效时延、是否切断使能和硬件优先级仍未知。`ReentrantCallbackGroup` 允许 action 并发（`:28-29`），主程序使用 4 线程（`:925-927`），没有 active-goal 互斥；多个轨迹可能竞争共享 CAN 和 `_trajectory_start_time`。并发、抢占和停止语义没有证据。

因此以下门禁仍未满足：真实型号/安装/TCP/映射、可靠设备时间戳、位置覆盖与周期、工作空间/碰撞审批、独立安全限幅/停止、shadow+故障注入、人工低速测试、独立实角/FK 验证。不得下发 PPO 动作。

## 7. 已确认、未知和下一决策门

**已确认：** 压缩包存在且可解压；URDF/SDF XML 可解析；静态链为 6 revolute；mesh 归档完整；ROS 节点有角度、位姿、直线和 FollowJointTrajectory 表面接口；状态可发布名义 30/120 Hz；二进制公开方法含角度读取、动态轨迹、停止 API。

**未知/阻塞：** 实物型号和安装、编码器映射、TCP、真实限位/速度/力矩、SDF/URDF canonical 选择、`.so` 内部时序/排队/覆盖/停止、真实状态率/延迟、Isaac Lab 可用环境、碰撞与桌面几何。

**下一决策门（需研究者/现场人员审批）：**

1. 确认私有包许可和实际硬件型号/法兰/TCP；提供不含密钥的安装和只读状态说明。
2. 审批在独立工作目录制作“适配版 URDF/SDF”，保留原始 hash 和逐项 diff；先 CPU XML/FK 校验。
3. 单独审批厂商 ROS/SDK 的只读编译与接口探测；在未完成安全门禁前不使能电机、不发运动命令。
4. 修复或重新选择 Isaac Lab/Isaac Sim 配对环境后，才进入 B1。

## 理解验收题

1. 为什么 URDF 的 `velocity/effort` 声明不能自动等于真实设备能力？
2. 为什么 action server 返回 `success` 不能替代关节状态反馈？
3. 为什么 10 ms 的 Actor forward 不代表 100 Hz 的真实闭环？
