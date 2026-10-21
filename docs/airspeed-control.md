# 简单空速控制

本项目提供可选的空速 PI：根据目标空速与测量空速的误差调整基础油门。源码默认关闭，手动模式始终为油门直通；俯仰仍由遥控和原有姿态控制器负责，没有高度环或 TECS。空速控制与油门到 PWM 的行程映射是两个独立环节。

## 使用方式

`airspeed_control_enabled=0`（源码默认值）时，手动和增稳均使用遥控基础油门。设为 1 后，A 主机在增稳模式、反馈和参数有效、杆量超过 2% 时运行空速闭环；手动模式仍为直通。B～G 不运行独立空速环，接收主机混控后的油门。SD 已保存的开关值会覆盖源码默认值。

当前 `runMasterController()` 只按增稳模式传入控制器的 `enabled`，没有把飞行锁定条件传入。因此锁定时仍可能运行 PI、更新滤波和积累积分；输出层将本机实际 PWM 强制设为 `pwm_channel4_trim`。不能将“锁定输出安全位置”理解为“空速环已停止或积分已清零”。身份无效或有待重启参数时，主循环会提前返回，不执行该次控制更新。

遥控归一化输入 -1～1 在准备目标时转换为 `manual_input[2]=(radio_channel3_normalized+1)/2`，并赋给 `targets.throttle`。手动混控直接读取 `manual_input[2]`，增稳油门控制器读取 `targets.throttle`。油门目标、手动基础油门和 PI 基础油门使用 0～1；混控差动叠加后可超出该范围，机间协议传输这个混控结果，最终由各机 PWM 输出层限幅。令 `u=targets.throttle`，闭环激活时的目标和基础输出为：

```text
V_target = airspeed_min + u * (airspeed_max - airspeed_min)
error = V_target - V_filtered
throttle = constrain(throttle_cruise + Kp_airspeed * error + integral, 0, 1)
```

仅在增稳且 `airspeed_control_enabled=1` 时，杆量不超过 2% 会把基础油门置为 0 并清除积分，即使此时反馈无效也如此。手动或开关为 0 时，近零杆仍按实际杆量直通，例如 0.01 输出基础油门 0.01。基础油门为 0 不保证编队各机最终输出最低 PWM：mixer 仍可能叠加偏航/滚转差动。

五路均直接调用 `actuatorCommandToPwm`，输入范围由 `pwm_channelX_mode` 决定：mode=0 接收 -1～1，按 min/trim/max 三点插值；mode=1 接收 0～1，按 min/max 两点插值，trim 仅决定上电、锁定及异常回退位置。没有单独的油门转换函数，也不自动调整调用方的指令范围。当前油门输出为 0～1，因此通道4应设置 mode=1；普通 1000～2000 μs 电调再配置 trim=1000、min=1000、max=2000、rev=1，最终命令 0/0.5/1 对应 1000/1500/2000 μs。mode 编译默认仍为 0，须设置并保存 `pwm_channel4_mode=1`；旧 SD 保存的模式也须核对。速度控制器不会修改安装参数，见 [执行器配置](firmware-guide.md#执行器锁定与限幅)。

## 空速与滤波

`airspeed` 是 MS4525 驱动经零偏扣除、负差压截零和固定密度 1.225 kg/m³ 换算的 m/s 值，保留未低通的数据供原有日志和诊断使用。它不是原始压力计数，也不是经过密度修正的真空速。

传感器约 50 Hz 采样；控制器按实际控制周期 `dt` 对保持的采样值做一阶低通，`tau=1/(2*pi*airspeed_lpf_hz)`，`alpha=dt/(tau+dt)`。首个有效值直接初始化滤波器。最近读取必须完整且状态为 0，样本年龄不超过 100 ms，数值有限并位于 0～100 m/s；无效反馈、无效控制周期（非有限、非正或超过 100 ms）及非法参数会退出闭环，清除积分并标记滤波器待重新初始化，退回遥控基础油门；增稳且开关为 1 时仍优先执行近零杆置零规则。

手动模式、开关为 0 或近零杆仅清除积分，不会因这些条件本身重置滤波器；只要反馈、周期和参数有效，滤波值仍持续更新。

此有效性检查不能识别空速管堵塞、安装误差或持续偏置；退回直通后，同一杆量重新表示油门，输出可能变化。

## 参数

均为本机 SD 参数，可在现有地面站修改并保存，下一周期生效；旧参数文件缺项时使用默认值。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `airspeed_control_enabled` | 0 | 0 直通；1 增稳时空速 PI |
| `airspeed_min` | 8 m/s | 最低目标空速，必须小于 `airspeed_max` |
| `airspeed_max` | 22 m/s | 最高目标空速；默认中杆目标为 15 m/s |
| `throttle_cruise` | 0.5 | PI 的基础油门 |
| `Kp_airspeed` | 0.04 | 比例增益，油门/(m/s) |
| `Ki_airspeed` | 0.01 | 积分增益，油门/((m/s)·s) |
| `airspeed_lpf_hz` | 2 Hz | 反馈低通截止频率；0 关闭滤波 |

`V_cruise` 仍单独用于偏航协调转弯，不决定速度环目标。速度积分项限制为 ±0.3；基础油门饱和时停止同方向积分，允许反向退出饱和。`integralControlEnabled=0` 或 `Ki_airspeed=0` 时清除 I 项。手动、近零杆、速度环禁用和无效反馈都会清除积分，退出这些条件后从零积累；锁定本身不会清除积分。改变飞行模式只有在后续实际执行控制器更新时才触发对应处理。

这些默认值仅为初始值，尚未经过实机调参。先根据实际机型确定目标范围和配平油门，再从 `Ki_airspeed=0` 调整比例，最后增加积分消除稳态误差。当前没有油门变化率限制或无扰切换；抗饱和针对基础油门，不反馈 mixer 差动和各机最终 PWM 的饱和状态。

日志增加 `throttleControlOutput` 下的 `airspeedTarget`、`filteredAirspeed`、`airspeedError`、`integralTerm`、`speedControlActive`，配合原始 `airspeed` 和 `throttleCommand` 检查闭环行为。未启用闭环时，目标、误差和积分发布为 0；有效反馈下仍更新滤波值。

## 软件验证

主机数值检查可直接使用 g++ 编译，不依赖 Arduino：

```powershell
New-Item -ItemType Directory -Force .cache | Out-Null
g++ -std=c++17 -O2 -ffast-math -I src test/control/throttle_controller_test.cpp src/control/throttle_controller.cpp src/control/control_output.cpp src/parameters/control_parameters.cpp -o .cache/throttle_controller_test.exe
if ($LASTEXITCODE -ne 0) { throw "Throttle test build failed" }
.\.cache\throttle_controller_test.exe
```

测试显式开启空速 PI，不依赖源码默认开关。覆盖目标映射、误差方向、上下饱和、积分限幅/清零、故障回退、低通、非法数值和带恒定阻力的一阶速度模型；该控制器单元测试不验证真实飞行锁定、机间通信或电调输出。固件构建使用 `pio run -e teensy41`。软件检查不代替实机验证。
