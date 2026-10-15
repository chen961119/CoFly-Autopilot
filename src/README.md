# 源码目录

项目头文件与实现均放在 `src`，同一模块的 `.h`、`.cpp` 存在同一目录。`main.cpp` 保留在根目录，仅编排初始化和主循环。

| 目录 | 职责 |
| --- | --- |
| `config` | 固件配置与运行时飞机身份 |
| `control` | 指令生成、目标归类、主/从控制、模式及运行状态 |
| `allocation` | 主/从 mixer、俯仰作动器映射、交联和指令融合 |
| `sensors` | 传感器采样、校准、滤波与姿态估计 |
| `actuators` | 执行器输出与 PWM 限制 |
| `interaircraft_comm` | 机间命令、状态接收与发送、级联转发 |
| `ground_station_service` | USB 地面站设备服务与数据接口 |
| `radio_comm` | 八路遥控输入与固定范围归一化 |
| `telemetry` | 遥测输出 |
| `system` | 时钟、飞行锁定与设备重启 |
| `diagnostics` | 诊断输出与 Teensy 固件调试 |
| `interface` | 按键与 OLED 人机界面 |
| `logging` | SD 飞行日志 |
| `math` | 数学辅助函数 |
| `parameters` | 参数注册、默认值、编解码、存储与 USB 参数服务 |

新增代码放入对应模块，需要新职责时建立新的模块目录；项目不再建立 `include` 文件夹。项目头文件引用统一使用相对 `src` 的路径，例如：

```cpp
#include "config/flight_config.h"
#include "control/control_modes.h"
#include "parameters/parameter_service.h"
```

PlatformIO 的 `include_dir = src` 与 `-I src` 提供统一头文件根目录，`build_src_filter = +<*>` 编译所有模块下的源码。普通和调试环境使用相同布局。调试脚本中的文件路径及主机测试中的头文件搜索路径须随文件迁移同步更新。

控制流程的七层职责和主从并列运行路径见 [控制架构](../docs/control-architecture.md)。原动态矩阵模型仍保留于 `control/control_allocation.cpp`，尚未接入当前固定系数混控。


## 统一惯性测量接口

`sensors/sensor_processing.h` 提供 `ang_vel[3]` 和 `ang_vel_filtered[3]`（deg/s，后者为传感器层统一滤波结果）、`ang_acc[3]`（deg/s²）和 `acc[3]`（g），数组按 X/Y/Z 排列。`sensors/filters.h` 提供 `att[3]`（deg，滚转/俯仰/偏航）和 `quat[4]`，按 W/X/Y/Z 排列，沿用现有姿态算法约定。内置 IMU 的 X 轴符号在传感器层统一到原控制和日志方向，外置 IMU 保持其安装轴映射。

主循环调用 `updateInertialSensors()`，按外置优先、否则当前 BMI088 路径进行采样和姿态估计，发布统一测量后计算角加速度。控制、日志、诊断、地面站和机间通信使用统一数组，具体型号的采样变量仅在 `sensors` 内部使用。

机间角速度为 `ang_vel_B[3]` 至 `ang_vel_G[3]`；现有协议仅传输滚转角速度，接收端只更新 `[0]`，`[1]`/`[2]` 保持 0，不能视为有效测量。协议包布局保持不变。SD 日志列名使用 `ang_vel[i]`、`ang_acc[i]`、`acc[i]` 和 `quat[i]`；编队日志合并了原内置/外置重复列，仅记录选定数据源。

五路应变测量统一通过 `strain_value[5]`（`int32_t` 数组）读取，索引 0～4 对应传感器通道 1～5；保持原报文解析和数值单位。SD 日志中的姿态和应变列分别使用 `att[i](deg)`、`strain_value[i]`。

主机目标由 `prepareControlMode()` 的主机分支更新全局 `targets`；主分配完成后，`generateSlaveCommand()` 更新 `slaveTargets[]`、`commands.slaves[]` 和独立的发送命令，路由包装结构已删除。`ControlCycleOutput commands` 仅含作动器结果，通信命令保存在独立的 `slaveCommands[]`。`prepareControlMode()` 更新各机的俯仰算法选择，并仅在主机设置遥控模式、策略、构型系数和刹车开关，控制器状态重置由各角色的 `run*Controller()` 处理。

主机手动模式的 `targets.roll/pitch/yaw` 为 ±1，手动控制器直接使用这些字段除以 2；增稳模式为角度/角速度目标。`targets.throttle` 为 0～1。从机俯仰目标保持度制，分配与日志需要角度时通过 `targetRollDegrees()` / `targetPitchDegrees()` / `targetYawDegrees()` 从 `targets` 换算，不再维护独立的全局目标或 manual 字段。

构型角目标统一使用 `targets.configuration[0～5]`，依次对应 AB、AC、BD、CE、DF、EG，初始值为零。指令生成每周期更新遥控轴目标，保留构型目标，供控制器直接修改并在后续周期继续使用。

构型控制输出系数 `coeffconfiguration` 由控制模式模块持有，初始值为 1；主机 `prepareControlMode()` 根据第 7 通道更新为 `1 + radio_channel7_normalized`，构型控制器直接读取该全局系数。

`control/control_output.h/.cpp` 按控制器持有全局 `manualControlOutput`、`rollControlOutput`、`pitchPidControlOutput`、`pitchIndiControlOutput`、`rollDampingControlOutput`、`yawControlOutput`、`throttleControlOutput`、`configurationControlOutput`；mixer、日志及遥测直接读取。分配结果仅由 `commands.local/slaves[]` 持有。模式模块持有全局 `lastMode`、`lastPitchController`；模式准备/接收阶段在覆盖当前值前保存旧值，控制入口仅判断切换。

主从控制周期均调用统一的 `prepareControlMode()`：主机读取遥控模式，从机接收上级命令，随后共同选择本机俯仰算法并计算模式状态。滚转阻尼在主机 `runMasterController()` 和从机 `mixSlaveCommands()` 内分别调用 `controlControllers().rollDamping.update()`，每周期沿所在角色路径执行一次。`mixSlaveCommands()` 与 `fuseSlaveCommands()` 均直接读写全局命令，无参数和返回值。

构型结果由 `controllers.configuration.update(targets.configuration)` 直接更新全局 `configurationControlOutput`，无返回值、临时结果或外部复制。主从机每周期调用以更新构型测量；只有主机增稳模式计算构型闭环，其他周期清零构型控制量并保持积分历史。

所有控制器的当前结果由各自 `update()` 直接发布，未启用时在该方法内清零当周期输出；外部入口只负责准备状态、选择控制器和调用。通用结果重置函数和滚转结果 getter 已删除。油门直通由 `ThrottleController::update()` 发布；INDI 的运动目标、舵机估计、诊断及 PWM 标定结果均在其 `update()` 中更新，分配层只转换结果尺度。

模式准备函数在更新本周期模式及俯仰算法后统一发布全局 `changed` 和 `integralEnabled`，主从控制入口直接读取，用于控制器状态重置与积分门控。
