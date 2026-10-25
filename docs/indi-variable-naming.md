# 俯仰 INDI：计算与变量约定

本机俯仰 INDI 由 [PitchIndiController](../src/control/pitch_indi_controller.cpp) 实现，主从机复用。它求解本机升降舵，不负责机间分配。俯仰抬头为正，升降舵下偏为正；全局发布的角度、角速度和角加速度分别用 rad、rad/s、rad/s²；控制器内部局部变量、历史状态和相关参数也采用 rad、rad/s、rad/s²，直接接收 SI 目标与反馈，不再在控制器内转换为度。

## 变量与发布接口

符号采用 INDI 文献常用含义，具体字段名为项目约定。`theta` 是欧拉俯仰角，`q` 是机体 Y 轴俯仰角速度，`r` 是机体 Z 轴角速度；`q` 不表示四元数或动压。

| 符号或后缀 | 含义 |
| --- | --- |
| `ref`、`cmd`、`meas` | 参考值、控制命令、测量反馈 |
| `f`、`hat`、`prev` | 滤波值、模型估计、上次有效更新的历史值 |
| `dot`、`delta_delta_e` | 时间导数、舵偏增量 Δδ_e |
| `raw` | 当前处理链中未滤波的量，不保证为传感器寄存器原值 |
| `rad`、`rad_s`、`rad_s2`、`us` | 弧度、弧度/秒、弧度/秒²、微秒 |
| `tau_*_s`、`alpha_*` | 时间常数（秒）、无量纲离散滤波系数 |
| `pwm_*_us` | 内部物理 PWM 命令或历史，单位 μs |

以下为 SI 全局 `pitchIndiControlOutput` 的直接成员（q 为 rad/s、q_dot 为 rad/s²、delta_e 为 rad），类型定义见 [control_types.h](../src/control/control_types.h)：

| 字段 | 含义 |
| --- | --- |
| `q_ref` | 运动学转换并限幅后的机体俯仰角速度参考 |
| `q_f` | INDI 专用低通后的 q 反馈 |
| `q_dot_ref` | 虚拟控制量：目标俯仰角加速度 |
| `q_dot_meas` | 用于增量反演的 `ang_acc[1]`，已由传感器层差分滤波 |
| `delta_e_cmd` | 限速、一次低通及通道3范围限幅后的命令舵偏 |
| `delta_e_hat` | 本周期推进舵机模型后的舵偏估计 |
| `indiOutput` | 模式0下为抬头正的归一化俯仰控制量，0 为配平，±1 为相应方向端点；主/从 mixer 取负后转换为下偏正舵面量，不是角加速度 |
| `isValid` | 本周期是否完成有效控制计算 |

旧日志字段 `desiredRateDegPerSec`、`desiredAngularAccelerationDegPerSec2`、`commandedElevatorDeflectionDeg` 分别对应上述 `q_ref`、`q_dot_ref`、`delta_e_cmd` 字段。自 2026-10-22 本次修改起，日志字段 `indiOutput` 从下偏正改为抬头正，数值与此前版本互为相反数；`delta_e_cmd`、`delta_e_hat` 仍为下偏正，物理输出不变。旧 `elevatorPwmUs` / `pwm_cmd_us` 是整数物理 PWM，不能直接按 `indiOutput` 的尺度读取；短暂采用两端归一化的历史版本也使用过 `indiOutput` 名称，读取历史日志必须区分固件版本。

## 控制律与处理顺序

以下控制律、滤波历史和数值限幅全部按 SI 解释，输出直接发布，不再进行度制换算。

令 `K_q=indi_pitch_q_gain`，`G_qe=indi_pitch_effectiveness`。默认 G_qe 为 -113.65，单位 (rad/s²)/rad；分子分母同步换单位，所以数值不变。限速默认 `Units::toRadians(800.0f)` rad/s，PWM 标定默认 `Units::toRadians(0.090909f)` rad/μs，参数加载直接采用当前弧度值，不自动转换旧值。INDI 的物理 PWM 归一化已经使用当前 min/trim/max，不能再套 PID 的 5 倍补偿。

```text
theta_ref_dot_raw = clamp((theta_ref - theta_ref_prev)/dt, -Units::toRadians(10.0f), Units::toRadians(10.0f))
theta_ref_dot_f   = LPF(theta_ref_dot_raw)
e_theta          = theta_ref - theta
theta_dot_cmd    = 3 * e_theta + theta_ref_dot_f
q_ref            = clamp((theta_dot_cmd + r_f*sin(phi))/cos(phi), -Units::toRadians(120.0f), Units::toRadians(120.0f))
q_dot_ref        = K_q * (q_ref - q_f)
delta_delta_e    = 0.7 * (q_dot_ref - q_dot_meas) / G_qe
delta_e_cmd      = delta_e_hat + delta_delta_e
```

三角函数直接使用弧度滚转角。之后依次：

1. 相对上次最终命令按 `indi_pitch_rate_limit_rad_s * dt` 限速。
2. 按通道3物理端点推导的角度范围限幅，在角度域低通一次，再按同一范围限幅。
3. 反算浮点物理 PWM，并按通道3范围限幅；控制器内不量化为整数 μs。
4. 将物理 PWM 写入 `pwm_delay_buffer_us`；读取延迟命令，换算舵角并推进一阶舵机模型。
5. 将限幅后的本机 PWM 逆映射并取负，发布抬头正的 `indiOutput`；主/从 mixer 选用时再取负，恢复下偏正的舵面量。交联、前馈、通信、融合及最终 PWM 映射均使用恢复后的舵面坐标。

增量公式使用进入本次更新时的 `delta_e_hat`；发布的 `delta_e_hat` 是模型更新后的值，供下次反演使用。目标变化率前馈不等于含角度误差修正的 `theta_dot_cmd`。`q_f` 和 `q_dot_meas` 使用两条既有滤波路径，不能假定完全时间对齐。

## 配平零点与线性换算

令 C 为通道3的零舵角 PWM：mode=0 时为 `pwm_channel3_trim`，mode=1 时为 `(pwm_servo3_min_us+pwm_servo3_max_us)/2`。s=`pwm_channel3_rev`（±1），k=`indi_pitch_pwm_to_rad_k`（正值，rad/μs）。模式 0 的换算为：

```text
delta_e    = (PWM-C) * s*k
PWM        = C + delta_e/(s*k)
span       = PWM >= C ? pwm_servo3_max_us-C : C-pwm_servo3_min_us
indiOutput = span > 0 ? -s*(PWM-C)/span : 0
elevator   = -indiOutput
```

C 是绝对 PWM，对应 0 rad。物理 PWM 到舵角仍使用同一个 k，与最终输出映射互逆。模式 0 要求 min≤trim≤max，逻辑量按中位两侧分别归一化。模式 1 接收 0～1，不使用 trim，逆变换先求 `u=(PWM-min)/(max-min)`，再按方向返回 `rev<0 ? 1-u : u`；物理中点对应 0.5，trim 仅为安全位置。零行程逆变换返回 0。例如范围 1200～1700 μs、trim=1500、rev=1 时，模式 0 的 -1/0/+1 对应 1200/1500/1700 μs，模式 1 的 0/0.5/1 对应 1200/1450/1700 μs；rev=-1 时两端交换。当前升降舵的手动/PID、交联和前馈融合使用有符号贡献，因此通道3应保持 mode=0；仅修改映射参数不会将整套混控转换为绝对位置坐标。

模式 1 的 `actuatorPwmToCommand()` 返回 0～1 绝对位置，INDI 出口取负后为 -1～0、中点为 -0.5，不具备模式 0 的抬头正有符号语义；mixer 再次取负仅恢复原绝对位置，不解决现有交联/融合对有符号贡献的要求。

角度上下限由两个物理 PWM 端点分别换算再排序，兼容 rev=-1。不再使用独立截距、舵角上下限或 INDI PWM 上下限；删除参数的兼容处理见 [参数迁移](parameter-storage.md#格式与介质迁移)。最终物理输出仍按 [固件指南](firmware-guide.md#执行器映射与方向迁移) 处理融合贡献、rev、trim 和限幅。

## 状态与延迟模型

`reset(当前目标)` 设置目标差分起点并标记重新初始化；下一次有效 update 将 64 个 PWM 历史填为 C，命令/估计置零，q 滤波器从当前测量开始。`r_filter` 保留历史。`enabled=false` 只清空本周期发布结果，不推进状态，也不代替 reset。

延迟步数为 `round(indi_pitch_servo_delay_s/dt)`，限制到 0～63；0 读取本周期命令，其他值读取此前命令。默认 30 ms 延迟在 dt=2 ms 时为 15 周期。延迟舵角经过时间常数 `indi_pitch_servo_tau_s` 的一阶响应；默认 15 ms。队列存储当时的物理 PWM，不按当前正负行程重新缩放历史。

## 模型与使用限制

舵偏估计只包含本机 INDI 命令，未包含之后从机融合加入的主机升降舵前馈，也未反馈融合后的最终通道饱和或锁定输出。因此 `delta_e_hat` 不是最终舵面的完整估计，更不是实测舵角。

例如通道范围 1000～2000 μs、rev=1、trim=1500、k=`Units::toRadians(0.090909f)`，本机目标与运动反馈均为零时，INDI 可保持输出和估计均为零；若主机持续加入 +0.1 前馈，最终 PWM 为 1550 μs，按标定对应约 `Units::toRadians(4.54545f)` rad 命令舵角，模型仍可为 0°。前馈只叠加一次，差别来自模型输入范围。

当前运动学计算直接除以 cos(phi)，未增加最小分母保护；在线改变 trim/rev/k 也不等同于实际舵面重新标定。边界收缩时物理范围约束可能优先于相邻周期限速。历史数值检查的范围与证据状态见 [研究与历史](archive/CONTROL_ARCHITECTURE_ANALYSIS.md#历史检查的证据边界)，不能由软件换算正确推断实机稳定性。
