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
| `ground_station_service` | USB / X6 地面站设备服务与数据接口 |
| `radio_comm` | 八路遥控输入与固定范围归一化 |
| `telemetry` | 3DR X6 双向地面站链路 |
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

`sensors/sensor_processing.h` 提供 `ang_vel[3]` 和 `ang_vel_filtered[3]`（rad/s，后者为传感器层统一滤波结果）、`ang_acc[3]`（rad/s²）和 `acc[3]`（m/s²），数组按 X/Y/Z 排列。`sensors/filters.h` 提供 `att[3]`（rad，滚转/俯仰/偏航）和 `quat[4]`，按 W/X/Y/Z 排列，沿用现有姿态算法约定。内置 IMU 的 X 轴符号在传感器层统一到原控制和日志方向，外置 IMU 保持其安装轴映射。

主循环调用 `updateInertialSensors()`，按外置优先、否则当前 BMI088 路径进行采样和姿态估计，发布统一测量后计算角加速度。控制、日志、诊断、地面站和机间通信使用统一数组，具体型号的采样变量仅在 `sensors` 内部使用。

机间角速度为 `ang_vel_B[3]` 至 `ang_vel_G[3]`；现有协议仅传输滚转角速度，接收端只更新 `[0]`，`[1]`/`[2]` 保持 0，不能视为有效测量。协议包布局保持不变。SD 日志列名使用 `ang_vel[i]`、`ang_acc[i]`、`acc[i]` 和 `quat[i]`；所有飞机使用统一日志格式，仅记录本机选定数据源及统一滤波结果。

五路应变测量统一通过 `strain_value[5]`（`int32_t` 数组）读取，索引 0～4 对应传感器通道 1～5；保持原报文解析和数值单位。SD 日志中的姿态和应变列分别使用 `att[i]`、`strain_value[i]`。

主机目标由 `prepareControlMode()` 的主机分支更新全局 `targets`；主分配完成后，`generateSlaveCommand()` 更新 `slaveTargets[]`、`commands.slaves[]` 和独立的发送命令，路由包装结构已删除。`ControlCycleOutput commands` 仅含作动器结果，通信命令保存在独立的 `slaveCommands[]`。`prepareControlMode()` 更新各机的俯仰算法选择，并仅在主机设置遥控模式、策略、构型系数和刹车开关，控制器状态重置由各角色的 `run*Controller()` 处理。

所有模式下 `targets.roll/pitch` 均为角度目标（rad），`targets.yawRate` 为偏航角速度目标（rad/s），`targets.throttle` 为 0～1。主机每周期将遥控输入换算为上述目标，切换模式不改变单位。手动直通输入另存为 `manual_input[4]`，顺序为 `[roll, pitch, throttle, yaw]`；roll/pitch/yaw 为 -1～1，throttle 为 0～1。手动混控及从机命令中的手动滚转贡献读取 `manual_input`，物理角度目标仍读取 `targets`。从机俯仰目标使用 rad，需要度制的调用位置直接使用 `Units::toDegrees()`。日志分别记录 `manual_input` 与 `targets`。

偏航角速度控制器为 `control/yaw_rate_pid_controller.h/.cpp` 中的 `YawRatePidController`，实例为 `controllers.yawRate`，结果为 `yawRateControlOutput`。内部增益使用 `Kp_yaw_rate` / `Ki_yaw_rate` / `Kd_yaw_rate`，最大角速度使用 `maxYawRate`；SD 和地面站参数键保留 `Kp_yaw` / `Ki_yaw` / `Kd_yaw` / `maxYaw`，继续兼容已保存的参数。

构型角目标统一使用 `targets.configuration[0～5]`，依次对应 AB、AC、BD、CE、DF、EG，初始值为零。主机 `prepareControlMode()` 每周期根据第 7 通道更新构型目标：归一化值 >0.2 时上反，左侧差值为 +10°、右侧为 -10°；<-0.2 时下反，符号相反；其余为零。每对相邻飞机的目标滚转差为 10°，外侧飞机相对 A 的目标角度逐级累加。

构型控制输出系数 `configurationControlScale` 由控制模式模块持有，固定为 `1.0f`，构型控制器直接读取该全局系数。

`control/control_output.h/.cpp` 按控制器持有全局 `rollControlOutput`、`pitchPidControlOutput`、`pitchIndiControlOutput`、`rollDampingControlOutput`、`yawRateControlOutput`、`throttleControlOutput`、`configurationControlOutput`；mixer、日志及遥测直接读取。分配结果仅由 `commands.local/slaves[]` 持有。模式模块持有全局 `previousFlightMode`、`previousPitchController`；模式准备/接收阶段在覆盖当前值前保存旧值，控制入口仅判断切换。

`pitchIndiControlOutput` 直接保存角速度目标与滤波反馈、角加速度目标与实测值、命令舵偏与估计舵偏，以及有效标志和归一化 `indiOutput`，不再嵌套 `diagnostics` 或重复保存同一过程量。全局命令/估计舵偏均为 rad、下偏为正；估计值来自舵机模型，并非舵角传感器。上述字段均写入统一 SD 日志。 INDI 变量按 `theta`（俯仰角）、`q`（机体俯仰角速度）、`q_dot_ref`（虚拟控制量/目标角加速度）、`delta_e`（升降舵偏角）命名，后缀和旧名对照见 [INDI 变量命名](../docs/indi-variable-naming.md)。

`ActuatorCommand` 的副翼和升降舵均为下偏正的归一化舵面偏移。手动/PID/INDI 的俯仰控制量统一为抬头正，在主/从俯仰分配入口取负，转换为下偏正的舵面量；通信与融合原样传递该舵面坐标。INDI 内部的舵偏与舵机模型仍为下偏正，仅发布的 `indiOutput` 在 PWM 逆映射后取负。俯仰角目标仍抬头为正，最终 rev 只适配各机安装方向。

主从控制周期均调用统一的 `prepareControlMode()`：主机读取遥控模式，从机接收上级命令，随后共同选择本机俯仰算法并计算模式状态。滚转阻尼在主机 `runMasterController()` 和从机 `mixSlaveCommands()` 内分别调用 `controlControllers().rollDamping.update()`，每周期沿所在角色路径执行一次。`mixSlaveCommands()` 与 `fuseSlaveCommands()` 均直接读写全局命令，无参数和返回值。

构型结果由 `controllers.configuration.update(targets.configuration)` 直接更新全局 `configurationControlOutput`，无返回值、临时结果或外部复制。主从机每周期调用以更新构型测量；只有主机增稳模式计算构型闭环，其他周期清零构型控制量并保持积分历史。

所有控制器的当前结果由各自 `update()` 直接发布，未启用时在该方法内清零当周期输出；外部入口只负责准备状态、选择控制器和调用。通用结果重置函数和滚转结果 getter 已删除。油门由 `ThrottleController::update()` 发布，主机增稳时默认执行空速低通与 PI，其余路径直通或清零，见 [简单空速控制](../docs/airspeed-control.md)；INDI 的运动目标、舵机估计和归一化 `indiOutput` 均在其 `update()` 中更新，控制器内部保留物理 PWM 命令和延迟队列，出口按通道映射逆变换后取负，主/从 mixer 再取负使用。模式 0 下 `indiOutput` 为抬头正、配平为零的控制量；日志中的该字段相对旧版反号，舵偏字段与实际 PWM 保持原方向。命令范围统一使用通道3上下限；计算、接口和模型限制见 [俯仰 INDI](../docs/indi-variable-naming.md)。

模式准备函数在更新本周期模式及俯仰算法后发布 `controlModeChanged`，供控制器重置状态。`ControlParameters::integralControlEnabled` 是本机 SD 整数参数（0 关闭、1 开启，默认 1），独立于手动/增稳模式；主从控制入口读取它进行 PID 积分门控，关闭时清除积分历史。

全局物理量的 SI 单位、命名迁移、兼容边界与逐处验证见 [全局变量 SI 迁移报告](../docs/global-si-migration.md)。日志直接记录 SI 物理量；时间与 PWM 单位保持原约定。
