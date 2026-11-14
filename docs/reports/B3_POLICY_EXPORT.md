# B3：策略导出与独立推理接口（SIM_ONLY）

> 状态：`PASS（SIM_ONLY 导出链路）`。这只表示最终 Actor 可独立加载并满足张量契约，不表示圆轨迹任务成功或具备实机部署资格。

## 接口固定项

- 输入：`float32` `(N,31)`，沿用 B2-S1 实际状态观测和环境内已定义的尺度；不使用真实硬件不可得的特权量。
- 输出：`float32` `(N,6)`，确定性 `tanh(mu)`，有限且属于 `[-1,1]`。环境再按 B2-S1 的统一动作映射执行位置增量、变化率限制、累积目标与关节限位裁剪。
- 网络：Actor 两层 128 单元 MLP，tanh 隐层；训练策略为 tanh-normal，导出 Actor 为确定性 mean actor。Critic 不导出到部署接口。
- 配置：[`configs/ppo/circle_ppo_sim_only.json`](../../configs/ppo/circle_ppo_sim_only.json) 固定 observation/action 维度、网络、动作分布和训练超参数；所有参数标记 `SIM_ONLY`。

## 文件与验证

- 训练 runner：[`scripts/train_ppo.py`](../../scripts/train_ppo.py)。保存 `checkpoint_latest.pt`，validation 更新保存 `checkpoint_update_XXXXX.pt`，最终导出 `actor.pt` TorchScript。
- 独立加载：[`scripts/infer_actor.py`](../../scripts/infer_actor.py) 在训练进程之外检查 `(1,31) → (1,6)`、有限值、边界和平均 forward 时间。
- Smoke 证据：`runs/b3_smoke/actor.pt` 可加载，`infer_actor.py` 已通过；summary 记录 CPU Actor forward 约 `0.050 ms`。该时间只表示纯网络 forward，不含 FK、观察构造、Isaac step、动作安全层或任何真实 SDK 通信。
- 正式 Actor 保留在本地忽略目录 `runs/b3_seed20261201/actor.pt`，SHA-256 为 `381dd7cae4d883f4f8908730c97b4cd58b72a0472f157dcae33bfff13e932b80`。独立 `scripts/infer_actor.py` 检查已通过，证据为 `runs/b3_seed20261201/infer.json`：输入 `(1,31)`、输出 `(1,6)`，输出范围约 `[-0.1497, 0.1727]`，平均 CPU forward `0.0346 ms`。训练 runner 的 `summary.json` 记录的是包含不同测量开销的 `0.0506 ms`，两者不是同一计时上下文。权重不上传公开仓库。

## 实机边界

导出成功不等于部署授权。B0 已确认真实 SDK 的连续目标覆盖、反馈时序、关节零位/方向、停止和安全层仍未验证；因此当前 Actor 只能用于独立 SIM_ONLY 评估，不得连接 ROS/CAN 或真实电机。

## 学习验收问题

1. 为什么导出 deterministic `tanh(mu)` 与训练时 tanh-normal 采样是两种不同用途？
2. 为什么必须保存 observation/action 配置、关节顺序和模型 hash，而不能只保存 `actor.pt`？
3. 纯 Actor forward 时间与完整 observation-to-command 延迟之间缺少哪些环节？
