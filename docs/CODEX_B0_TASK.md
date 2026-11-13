# 给 Codex 的第一条任务｜B0 静态模型与控制接口审查

> **范围：只读 + 有限的纯 Python/CPU 自检。不安装大型依赖，不启动 Isaac、PPO、GPU 任务；不连接或运动真实机器人。**

## 上下文
使用实验室当前六自由度机械臂完成**无相机、无接触、固定平面空中小圆形轨迹跟踪 PPO Sim2Real**。研究目标是最小完整训练→实机流程，禁止扩展到抓取、避障、视觉、Residual RL 比较或新算法。用户会在其本地提供官方 `student_ros.zip`（或解压后的厂商模型包），它不是仓库中的可公开资产。

## 要求
1. 审查本地宿主环境：OS、Python、ROS、Isaac Sim / Lab、GPU/显存、已有虚拟环境、CUDA，**记录**，不升级。
2. 审查完整厂商模型包：URDF/Xacro、mesh 路径有效性、links/joints、关节方向/上下界、固定零位、质量/惯量、collision 与 visual、坐标系和 TCP，抽取到报告；对缺失参数标 UNKNOWN，禁止填入虚构速度/力矩上限。
3. 审查 ROS 2/SDK：有哪些**真正的**位置/速度/力矩命令接口？实际机器人反馈何种状态？预期状态率与控制率来自代码还是实验？控制指令是否覆盖上一条、会否排队、何时认定执行完毕？底层是否有超时与限速？SDK 是否是二进制黑盒？
4. 用只读代码定位电机编码器角度到 URDF 关节角度的 offset、符号、关节名顺序，标出需要实机确认的变换。
5. 给出 2 种候选动作协议：`bounded_delta_joint_target`（优先）以及仅在底层支持时的低频分段动作。列出它们能否真实闭环，缺证据写 UNKNOWN。**不得将预录轨迹播放称作 PPO 实时闭环。**
6. 离线分析潜在空中圆轨迹的参数选取方法：工作空间、桌面/底座/自碰撞、方向、TCP、关节限位留量。此阶段先生成“参数选取算法”，不要写未核实的实机圆心/半径/周期。
7. 审查厂商文件的许可证/再分发限制，并确认 `.gitignore` 不会上传 SDK 或私有文件。

## 输出
- `docs/reports/B0_MODEL_AND_SDK_AUDIT.md`：逐项证据，路径、代码行号、结论（PASS/PARTIAL/UNKNOWN）。
- `docs/reports/B0_INTERFACE_MATRIX.md`：command、feedback、频率、是否可流式更新、stop、风险和证据来源。
- 如有必要，新增**纯离线**审查脚本和 CPU 测试（须不会修改厂家文件）。
- 更新 `docs/STATUS.md`；不擅自宣布 B0 PASS。
- 报告提出下一阶段 Isaac 导入与控制接口试验的**审批请求**；不要自动继续。

## 禁止事项
- 不进行 ROS 真实运动测试或电机使能。
- 不直接将模型文件/SDK 上传到公开仓库。
- 不调节系统驱动或重装 Isaac / ROS。
- 不做 PPO 训练、reward 调参、Domain Randomization 和 sim2real 的成功声明。
- 不跨项目修改用户已有 Factory/FR3/ManiSkill 仓库。

## 理解验收（提交给研究者回答）
- 为什么 URDF 的 velocity/effort 声明不能自动等于真实设备能力？
- 为什么 Action server 发送 `success` 不能替代关节状态反馈？
- 为什么一个 10 ms 的 Actor forward 不代表 100 Hz 的真实闭环？
