# Policy↔Robot 接口契约（草案，B0/B1 后冻结）

目前仅是约定模板；所有维度、单位、边界、频率和零位必须与**已验证的真实控制接口**一致后冻结。

## Reference
圆形参考轨迹位于经过确认的 robot base frame 的 XY 平面：

\[p_d(t)=[x_c+r\cos(\omega t),\ y_c+r\sin(\omega t),\ z_c]\]

`r>0`，`ω=2π/T`。实际参数不能在 URDF、SDK、安全空间审查前确定。FK 的 `TCP` 必须与真实末端参考点一致。

## Observation 候选

\[o_t=[q_t, \hat{\dot q}_t, p_d(t)-p_{ee}(t), v_d(t), a_{t-1}]\]

如果 6 轴机械臂，维数暂算 `6 + 6 + 3 + 3 + 6 = 24`。这里 `v_d(t)` 提供轨迹前进方向，避免同一空间点处的时间歧义。

- `q_t` 单位 rad，`dq_t` 单位 rad/s，关节顺序必须固定。
- `p_ee` 由当前 joint state + FK 计算，坐标系为 base frame；不需要相机。
- 速度未被硬件反馈时应标记为**估计**，使用实际状态时间戳计算，且定义无效时间窗口。
- 绝不可把“仿真 ground-truth 的真实不可观测量”作为 Actor 输入。

## Action 候选

Actor 输出 `a∈[-1,1]^6`，独立包装器将其转换为有边界的**关节位置参考值**；最终映射形式（相对实测 q / 相对旧 reference）由 B0 控制协议决定。必须记录：raw action、scaled action、safety-clipped action、commanded target、observed q。

## 时序（未冻结）

`physics_dt`、`policy_dt`、`servo_dt`、`state_publish_dt`、`state_delivery_dt` 均为不同量。记录 `observation_timestamp`、`policy_start/end`、`command_send`、`feedback_receive`，不得把推理耗时写作端到端控制频率。

## 安全

先限定安全姿态与工作空间，再做自碰撞/环境碰撞、关节软/硬限位、速度、加速度、连续变化幅度与超时门禁；所有拒绝原因可诊断。正式参数来自厂商和现场确认，不写死臆造阈值。
