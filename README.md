# Robot RL Sim2Real｜机械臂空中轨迹跟踪

> 状态：`SCAFFOLD_READY / B0_PENDING`（2026-11-13）  
> 目标：用实验室真实机械臂完成**一次从模型审查、Isaac Lab PPO 训练到真实机械臂低速闭环执行**的完整 Sim2Real。  
> **本仓库目前只有工程骨架和纯数学轨迹生成器，没有训练结果，也没有实机控制能力。**

## 首个任务（任务范围已冻结，参数尚未冻结）

**无相机、无吸盘、无夹爪、无接触的空中圆形轨迹跟踪。**

- 给定一个位于真实机器人安全工作空间内的圆形参考轨迹。
- PPO Actor 根据关节状态、FK 估计的末端状态及当前/未来参考轨迹产生受限动作。
- 由真实机器人**已有且经过验证**的底层控制器负责关节伺服。
- 使用独立安全层约束并记录动作，不将网络原始输出直接发送硬件。
- 先完成一个小半径、低速圆形轨迹的训练与部署，再讨论不同轨迹或泛化。
- 对照一个传统控制器，比较仿真与实机的轨迹误差、动作平滑性和指令时序。

**任务目的：学习真实 Sim2Real 流程；不研究 PPO 与 Residual RL 的优劣，也不追求算法创新。**

### 当前硬件线索

用户提供的厂商 ROS/URDF 压缩包曾被初步识别为 Episode1 系列 6-DoF 机械臂；机器人型号、关节零位、动力学、SDK 控制频率与安全边界均需 B0 实机/官方材料核验。**不是 FR3，不得移用 FR3 或 Panda 的模型与参数。**

厂商提供的 URDF / mesh / SDK 不会自动被提交到仓库。仅在核实许可与访问策略后再决定如何存放。

## 项目阶段

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| B0 | 厂商模型、关节映射、控制接口和安全可行性审查 | `PENDING` |
| B1 | Isaac Lab 导入与非策略动作验证 | `NOT_STARTED` |
| B2 | 轨迹跟踪 MDP + 基线控制 + 零动作/随机策略检查 | `NOT_STARTED` |
| B3 | PPO 训练（smoke→正式） | `NOT_STARTED` |
| B4 | 未见轨迹/初态仿真独立评估 | `NOT_STARTED` |
| B5 | 推理导出、接口契约、影子推理与故障注入 | `NOT_STARTED` |
| B6 | 现场授权下的低速实机 PPO 闭环 | `NOT_STARTED` |
| B7 | 仿真/实机比较、问题复盘和项目验收 | `NOT_STARTED` |

详情见 [路线图](docs/ROADMAP.md)、[状态与证据](docs/STATUS.md) 和 [Codex 第一步任务](docs/CODEX_B0_TASK.md)。

## 工程目录

```text
robot-rl-sim2real/
├── AGENTS.md                  # Codex 行为与安全边界
├── pyproject.toml             # 仅含纯 Python 的独立基础包
├── configs/                   # 协议模板；具体参数等待 B0 验证
├── docs/
│   ├── PROJECT_CHARTER.md     # 任务定义与非目标
│   ├── ROADMAP.md             # 阶段门禁
│   ├── STATUS.md              # 当前真实进度
│   ├── CODEX_B0_TASK.md       # 给 Codex 的第一条指令
│   ├── INTERFACE_CONTRACT.md  # Actor/控制器/日志接口草案
│   ├── SAFETY_GATE.md         # 实机硬门禁
│   └── EXPERIMENT_PROTOCOL.md # 验证方法和指标
├── model_assets/README.md    # 本地厂商模型存放规则（模型本体不提交）
├── src/robot_rl_sim2real/    # 纯数学轨迹生成器（不是策略）
├── scripts/                   # 只读参考轨迹 smoke
├── tests/                     # 无机器人、无 Isaac 的 CPU 单元测试
└── .github/workflows/         # 基础 Python CI，不会接触机器人
```

## 现在可以运行的唯一测试

```bash
python3 -m unittest discover -s tests -v
PYTHONPATH=src python3 scripts/preview_reference.py
```

该测试只验证数学圆形轨迹输出，**不访问 ROS/Isaac/硬件，也不证明 RL 或 Sim2Real 成功**。

## 下一步（Codex）

先执行 [B0 只读审查任务](docs/CODEX_B0_TASK.md)，明确证据与控制能力，再由研究者审批 Isaac Lab 环境搭建。不要直接开始 PPO 或机械臂运动。

## 发布说明

仓库初始建议 **Private**。包括厂商资产、SDK 二进制包、机器人真实 IP 地址、认证信息和未获许可的视频在内的资料，不得未经审查公开。若后续需要开源简历项目，可把可公开的自研代码和脱敏结果单独整理。
