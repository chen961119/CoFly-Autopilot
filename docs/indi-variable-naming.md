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
| `q_f` | 本周期实际使用的统一角速度反馈 `ang_vel_filtered[1]` |
| `q_dot_ref` | 虚拟控制量：目标俯仰角加速度 |
| `q_dot_meas` | 用于增量反演的 `ang_acc[1]`，已由传感器层差分滤波 |
| `delta_e_cmd` | 限速、一次低通及通道3范围限幅后的命令舵偏 |
| `delta_e_hat` | 本周期推进舵机模型后的舵偏估计 |
| `indiOutput` | 模式0下为抬头正的归一化俯仰控制量，0 为配平，±1 为相应方向端点；主/从 mixer 取负后转换为下偏正舵面量，不是角加速度 |
| `isValid` | 本周期是否完成有效控制计算 |

旧日志字段 `desiredRateDegPerSec`、`desiredAngularAccelerationDegPerSec2`、`commandedElevatorDeflectionDeg` 分别对应上述 `q_ref`、`q_dot_ref`、`delta_e_cmd` 字段。自 2026-10-22 本次修改起，日志字段 `indiOutput` 从下偏正改为抬头正，数值与此前版本互为相反数；`delta_e_cmd`、`delta_e_hat` 仍为下偏正，物理输出不变。旧 `elevatorPwmUs` / `pwm_cmd_us` 是整数物理 PWM，不能直接按 `indiOutput` 的尺度读取；短暂采用两端归一化的历史版本也使用过 `indiOutput` 名称，读取历史日志必须区分固件版本。

## 控制律与处理顺序

以下控制律、滤波历史和数值限幅全部按 SI 解释，输出直接发布，不再进行度制换算。

令 `K_q=indi_pitch_q_gain`，`G_qe=indi_pitch_effectiveness`。默认 G_qe 为 -113.65，单位 (rad/s²)/rad；分子分母同步换单位，所以数值不变。舵偏限速默认为 800°/s（约 13.96 rad/s）。上下偏采用通道 3 的分段标定。INDI 的物理 PWM 归一化已经使用当前 min/trim/max，不能再套 PID 的 5 倍补偿。

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
2. 按上、下两侧的可用舵角限幅，在角度域低通一次，再按同一范围限幅。
3. 反算浮点物理 PWM，并按通道3范围限幅；控制器内不量化为整数 μs。
4. 将物理 PWM 写入 `pwm_delay_buffer_us`；读取延迟命令，换算舵角并推进一阶舵机模型。
5. 将限幅后的本机 PWM 逆映射并取负，发布抬头正的 `indiOutput`；主/从 mixer 选用时再取负，恢复下偏正的舵面量。交联、前馈、通信、融合及最终 PWM 映射均使用恢复后的舵面坐标。

增量公式使用进入本次更新时的 `delta_e_hat`；发布的 `delta_e_hat` 是模型更新后的值，供下次反演使用。目标变化率前馈不等于含角度误差修正的 `theta_dot_cmd`。`q_dot_meas` 取自传感器层对同一俯仰角速度反馈的差分，差分后的 20 Hz 低通会引入额外相位滞后。

## 升降舵三点标定

2026-11-12 白鹿原“刚性连接柔性机翼”试验的 A、B、C 三份 `params.cfg` 第 101–105 行记录了舵机输出通道 3 的最小 PWM、绝对配平、最大 PWM、安装方向和映射模式。三机均为 `rev=+1`、`mode=0`。按现场给出的近似舵角，设置以下三个标定点。舵角以下偏为正。

| 飞机（编号） | 上打约 15°（−0.2618 rad） | 配平（0 rad） | 下打约 20°（+0.3491 rad） |
| --- | ---: | ---: | ---: |
| A（1） | 1350 μs | 1580 μs | 1900 μs |
| B（2） | 1213 μs | 1439 μs | 1750 μs |
| C（3） | 1033 μs | 1436 μs | 1800 μs |

`indi_pitch_up_rad` 和 `indi_pitch_down_rad` 分别存上、下偏角的正幅值，单位为 rad。对已配置的 A/B/C 身份，出厂值分别取上表的 15° 和 20°。其余身份的两项默认为 0。某一侧设为 0 时，仅该侧用 `indi_pitch_pwm_to_rad_k × 该侧 PWM 行程` 求角行程，旧斜率默认为 0.00158666 rad/μs。如果一侧的 PWM 端点与配平重合，该侧可用舵角为零。各机试飞参数第 116 行均为 `pitch_controller=0`（PID）。在需要使用 INDI 的节点将此项设为 1，进入增稳模式后生效。

令 `P0` 为配平脉宽，在模式 0 取 `pwm_channel3_trim`。令 `Pu` 和 `Pd` 分别为上打和下打的 PWM 端点，安装方向为 +1 时依次取通道最小值和最大值，为 −1 时依次取最大值和最小值。设 `U`、`D` 为两侧的可用角度幅值。对已限制在 [−U, +D] 的舵角，分段换算如下。

```text
上打   PWM = P0 + (Pu - P0) × (−delta_e / U)
配平   PWM = P0
下打   PWM = P0 + (Pd - P0) × ( delta_e / D)
```

延迟队列中的脉宽使用同样的三点关系反算舵角。例如，A 机上打 7.5° 对应 1465 μs，下打 10° 对应 1740 μs。逻辑升降舵量在两个端点分别为 −1 和 +1，INDI 发布的抬头正控制量与之反号。主/从混控器将其转换回舵面坐标后叠加前馈，最后由执行器写出本机限幅后的 PWM。

现有手动、PID、交联和前馈使用以配平为零的有符号贡献，升降舵应配置成模式 0。模式 1 的零舵角位于 PWM 两端中点，通道指令范围为 0～1，INDI 接口中点对应 −0.5。

### 与建库代码的对应关系

初始提交 `c937f38` 的 `src/main.cpp::PITCH_INDI_control()` 在函数中按度计算增量反演，先对称限制舵角到 ±20°，再用固定斜率 0.090909°/μs 和截距 −136.3636° 换成 PWM，内部脉宽限制为 1100～1920 μs。按该公式，±20° 约对应 1280～1720 μs。当前控制器沿用 0.7 倍增量、舵偏限速、命令低通、纯延迟和一阶舵机模型。它从传感器层接收弧度制反馈，使用本机通道 3 的三点标定，将归一化俯仰控制量交给现有混控器。最终 PWM 受本机通道软件行程约束。

在主机上对照运行上述两个版本，令旧版从零舵角起步，并向两版输入相同的目标、姿态、已滤波俯仰角速度和角加速度。采用 2 ms 步长，A/B/C 各运行 2000 个未饱和周期，峰值舵令约 1.19°。两版命令舵角最大差约 0.00011°。端点、反装、混控饱和与舵机延迟的回归可在安装 C++17 编译器后运行 `python tools/control_regression/run_pitch_indi.py`。

## 状态与延迟模型

`reset(当前目标)` 设置目标差分起点并标记重新初始化。下一次有效 update 将 64 个 PWM 历史填为零舵角脉宽，命令与估计置零。传感器层在所有模式下持续更新角速度反馈，INDI 启用后直接读取当期的 q 和 r。`enabled=false` 时清空本周期发布结果。

延迟步数为 `round(indi_pitch_servo_delay_s/dt)`，限制到 0～63；0 读取本周期命令，其他值读取此前命令。默认 30 ms 延迟在 dt=2 ms 时为 15 周期。延迟舵角经过时间常数 `indi_pitch_servo_tau_s` 的一阶响应；默认 15 ms。队列存储当时的物理 PWM。读取时按当前标定换算，在线收窄行程后，旧脉宽按当前端点限幅。

## 模型与使用限制

舵偏估计只包含本机 INDI 命令，未包含之后从机融合加入的主机升降舵前馈，也未反馈融合后的最终通道饱和或锁定输出。因此 `delta_e_hat` 不是最终舵面的完整估计，更不是实测舵角。

例如 B 机取表中标定，本机目标和反馈为零时，INDI 可保持零舵偏估计。若主机持续加入 +0.1 升降舵前馈，最终指令对应下打约 2°，量化前为 1470.1 μs，而模型仍可保持 0°。

当前运动学计算直接除以 cos(phi)。在线改变配平、方向、角行程或后备斜率会立即改变 PWM 与舵角的换算关系。边界收缩时物理范围约束可能优先于相邻周期限速。历史数值检查的范围与证据状态见 [研究与历史](archive/CONTROL_ARCHITECTURE_ANALYSIS.md#历史检查的证据边界)。
