# 简单空速控制

PX4 固定翼 [Stabilized 模式](https://docs.px4.io/main/en/flight_modes_fw/stabilized) 的油门是直通。本项目在现有姿态增稳基础上增加独立的空速 PI，仅通过基础油门控制速度；俯仰仍由遥控和原有姿态控制器负责，不引入高度环或 TECS。

## 使用方式

`airspeed_control_enabled=1`（默认）时，A 主机在增稳且解锁后使用空速闭环；手动模式始终直通。设为 0 可恢复增稳油门直通。B～G 不运行独立速度环，继续接收主机混控后的油门。

油门杆 `u` 沿用 `targets.throttle` 的 0～1 范围。闭环目标和输出为：

```text
V_target = airspeed_min + u * (airspeed_max - airspeed_min)
error = V_target - V_filtered
throttle = constrain(throttle_cruise + Kp_airspeed * error + integral, 0, 1)
```

杆量不超过 2% 时基础油门为 0，并清除积分。注意这只切断速度环的基础油门：编队 mixer 随后仍可能叠加偏航/滚转差动，最终输出继续受原有锁定及 PWM 限幅约束。

油门保留现有逻辑尺度及 `PWM=1500+trim+rev*1000*command` 映射；如电调使用 1000～2000 μs 输入，应按现有 [执行器配置](firmware-guide.md#执行器锁定与限幅) 设置油门 trim。默认 trim=0 时，逻辑 0.5 已到 2000 μs，上半段会被限幅；速度控制器不会自动修改本机或从机的安装参数。

## 空速与滤波

`airspeed` 是 MS4525 驱动经零偏扣除、负差压截零和固定密度 1.225 kg/m³ 换算的 m/s 值，保留未低通的数据供原有日志和诊断使用。它不是原始压力计数，也不是经过密度修正的真空速。

传感器约 50 Hz 采样；控制器按实际控制周期 `dt` 对保持的采样值做一阶低通，`tau=1/(2*pi*airspeed_lpf_hz)`，`alpha=dt/(tau+dt)`。首个有效值直接初始化滤波器。最近读取必须完整且状态为 0，样本年龄不超过 100 ms，数值有限并位于 0～100 m/s；故障时立即退出闭环，清积分和滤波历史，退回遥控油门直通。无效控制周期（非有限、非正或超过 100 ms）及非法参数同样退回直通。

此有效性检查不能识别空速管堵塞、安装误差或持续偏置；退回直通后，同一杆量重新表示油门，输出可能变化。

## 参数

均为本机 SD 参数，可在现有地面站修改并保存，下一周期生效；旧参数文件缺项时使用默认值。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `airspeed_control_enabled` | 1 | 0 直通；1 增稳时空速 PI |
| `airspeed_min` | 8 m/s | 最低目标空速，必须小于 `airspeed_max` |
| `airspeed_max` | 22 m/s | 最高目标空速；默认中杆目标为 15 m/s |
| `throttle_cruise` | 0.5 | PI 的基础油门 |
| `Kp_airspeed` | 0.04 | 比例增益，油门/(m/s) |
| `Ki_airspeed` | 0.01 | 积分增益，油门/((m/s)·s) |
| `airspeed_lpf_hz` | 2 Hz | 反馈低通截止频率；0 关闭滤波 |

`V_cruise` 仍单独用于偏航协调转弯，不决定速度环目标。速度积分项限制为 ±0.3；基础油门饱和时停止同方向积分，允许反向退出饱和。`integralControlEnabled=0` 或 `Ki_airspeed=0` 时清除 I 项。手动、锁定、近零杆、速度环禁用和无效反馈都会清除积分，重新启用时从零开始。

这些默认值仅为初始值，尚未经过实机调参。先根据实际机型确定目标范围和配平油门，再从 `Ki_airspeed=0` 调整比例，最后增加积分消除稳态误差。当前没有油门变化率限制或无扰切换；抗饱和针对基础油门，不反馈 mixer 差动和各机最终 PWM 的饱和状态。

日志增加 `throttleControlOutput` 下的 `airspeedTarget`、`filteredAirspeed`、`airspeedError`、`integralTerm`、`speedControlActive`，配合原始 `airspeed` 和 `throttleCommand` 检查闭环行为。未启用闭环时，目标、误差和积分发布为 0；有效反馈下仍更新滤波值。

## 软件验证

主机数值检查可直接使用 g++ 编译，不依赖 Arduino：

```powershell
g++ -std=c++17 -O2 -ffast-math -I src test/control/throttle_controller_test.cpp src/control/throttle_controller.cpp src/control/control_output.cpp src/parameters/control_parameters.cpp -o .cache/throttle_controller_test.exe
.cache/throttle_controller_test.exe
```

覆盖目标映射、误差方向、上下饱和、积分限幅/清零、故障回退、低通、非法数值和带恒定阻力的一阶速度模型。固件构建使用 `pio run -e teensy41`。软件检查不代替实机验证。
