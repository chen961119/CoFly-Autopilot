# CoFly Autopilot 固件指南

本文说明当前 Teensy 4.1 / A–G 级联固件的配置、接口和执行器行为。项目定位与快速开始见 [仓库首页](../README.md)。

## 当前固件配置

当前 `src/config/flight_config.h` 启用以下条件编译宏：

| 配置 | 当前值 | 说明 |
| --- | --- | --- |
| 飞机身份 | EEPROM `aircraft_id` | 默认 0 未配置，1=A～7=G，重启生效 |
| 组合规模 | `COFLY_AIRCRAFT_COUNT` | 在 `src/config/aircraft_config.h` 定义，默认 3，支持 1/3/4/5/7，修改后重新构建 |
| 工作方式 | `COFLY_AIRCRAFT_COUNT == 1` | 1 为单机，其余为编队；`TEAM` 保留为存储配置标识 |
| 奇偶构型 | `aircraftCount() % 2` | 从编译期数量计算 |
| IMU 路径 | `INTIMU` | 使用本机 IMU |
| 接收机 | `USE_SBUS_RX` | 使用 SBUS 接收机 |
| 俯仰控制 | SD 参数 `pitch_controller` | 0 PID、1 INDI，默认 1，增稳模式适用 |
| 机型配置 | `expensive` | 启用贵飞机专用限幅和启动逻辑 |

同一数量及硬件/算法配置下，各节点共用同一个固件，通过 USB 设置身份并重启。数量使用编译宏，不在参数页出现；单机将 `COFLY_AIRCRAFT_COUNT` 设为 1 后重新构建，设置 `aircraft_id=1`。已保存增益保留，切换后应确认调参。存储声明与校验见 [统一参数与持久存储](parameter-storage.md)。

## 开发环境

- 开发板：Teensy 4.1
- MCU：NXP i.MX RT1062，600 MHz
- 框架：Arduino/Teensyduino
- 构建系统：PlatformIO
- 编译环境：`teensy41`
- 上传协议：`teensy-gui`
- 串口监视器：921600 baud

固件依赖由 `platformio.ini` 和 `lib/` 管理；电脑参数软件的 Python 依赖单独位于 `tools/ground_station/requirements.txt`。

## VS Code Teensy 板上断点调试

连接 Teensy，打开“运行和调试”，选择“TeensyDebug：编译、上传并调试”，按 F5。调试固件通过同一根 USB 线提供两个虚拟串口：`Serial` 保留地面站通信和普通输出，`SerialUSB1` 用于 GDB 设置断点和查看变量；任务终端显示各自 COM 号。初始暂停后按 F5 继续，再连接地面站；断点暂停期间地面站请求也暂停处理。普通 `teensy41` 固件保持单串口。详见 [板上调试说明](../tools/teensy_debug/README.md)。
克隆到新电脑后，先执行一次 PlatformIO 构建。本机的 `.vscode` 调试配置会自动生成，不纳入 Git。

## MT6701 夹角传感器

通过 `Wire1` 接入（SDA=17、SCL=16，7 位地址 `0x06`），与 OLED 和空速传感器共用 I²C。驱动位于 `lib/MT6701/`，按提供的示例先读 `0x03` 再读 `0x04`，将 14 位无符号值换算为 0～360°。`sensor_processing.cpp` 负责初始化、每周期采样、零点和安装方向，主函数只调用初始化与读取接口。

`relativeAngle_ready` 是 MT6701 减去本机零点后的测量夹角，跨 0°/360° 时转换到 ±180°；A/B/D/F 反向，C/E/G 正向。按键清零经统一参数服务保存 `mt6701_zero`（默认 EEPROM）；更换传感器后应重新清零。读取失败时保留最后有效值，每 20 ms 重试且禁止保存错误零点。MT6701 仅用于测量与 SD 日志记录。

默认每控制周期读取一次，无额外延时或串口输出。构型控制、控制分配、OLED `Relat`、USB/数传构型与机间相对角恢复使用 tag0.1 默认分支的相邻 IMU 滚转差，移除 `userotatesensor` 控制分支。OLED 没有有效下游姿态时显示 `--`。地面站完整构型需要连接 A 主机；机翼按 B—A—C 等物理顺序连接。编译与驱动模拟测试已通过，接线、安装方向和实际循环耗时需要板上确认。

## USB 地面站

应用提供通讯连接、室内/室外配色与字体、参数、传感器、遥控器和飞行数据页。新增地面站协议与参数协议共用 USB Serial；传感器校准与设置暂为明确返回 `NOT_IMPLEMENTED` 的接口，模式由代码直接判断遥控通道。使用与实现范围见 [地面站说明](../tools/ground_station/README.md)。

电脑端软件位于 `tools/ground_station/`，提供参数读取/就地修改（最多支持 512 项）、EEPROM/SD 按项保存、重启生效提示、列排序及支持十六进制显示的串口助手。参数类型显示为 `float` / `int`；双击当前值编辑，Enter 保存，Esc 取消。双击 `tools/ground_station/dist/CoFlyGroundStation.exe` 可启动；源码启动、协议、SD 恢复见 [参数软件使用说明](../tools/ground_station/README.md)。飞控开机在 SD 初始化后自动加载参数，立即生效参数修改成功后下次控制周期使用新值；身份重启生效，总数量由编译宏决定。

## PWM 中位与安装微调

`src/config/flight_config.h` 中统一定义 `PWM_CENTER_US = 1500`。遥控滚转、俯仰、偏航归一化以及通道 7 构型系数也以 1500 为零点；同一个遥控 PWM 数值产生的期望量会随中位修改而变化。

安装 trim 仅保留本机 `pwm_channel1_trim`～`pwm_channel5_trim`，编译默认值全部为 0 μs。各飞机通过 USB 参数表校准，在自己的 SD 卡中保存并于开机恢复；不再按机体宏选择 trim。机械中位为 `1500 + trim`，trim 不随 rev 反向。当前油门 PWM 转换以 `PWM_CENTER_US` 为基准，最终输出还受本机 rev、trim、锁定和限幅影响；遥控油门归一化见 `control/control_modes.cpp::prepareControlMode()`。

解锁后的有效控制输出在 `applyAndTransmitActuatorCommands()` 中按各路 `PWM_SERVO1_MIN_US` / `PWM_SERVO1_MAX_US` 至 `PWM_SERVO5_MIN_US` / `PWM_SERVO5_MAX_US` 独立限幅，默认各路均为 1000～2000 μs；锁定输出不受此软件限幅影响。Servo 驱动范围由 `PWM_SERVO_ATTACH_MIN_US` / `PWM_SERVO_ATTACH_MAX_US` 独立设置，当前为 1000～2000 μs。俯仰 INDI 还具有独立的可调物理 PWM 限幅。INDI 的绝对 PWM 到舵角标定继续使用原拟合系数，其中立初始化使用 `PWM_CENTER_US + pwm_channel3_trim`。

所有飞机的升降舵到副翼补偿统一使用本地俯仰控制偏移除以 15；INDI 使用实测物理 PWM，因此需扣除本机机械中位并还原控制方向。机械 trim 不参与控制补偿，零控制量时补偿为零。日志中的五路执行器输出仅记录 `ActuatorOutput[0..4]`，包含本机 trim、锁定处理和最终限幅；子机状态回传滚转、俯仰、偏航，不记录子机物理升降舵 PWM。

迁移对比使用旧版 `scaleCommands()`、当前 `convertControlCommandsToPWM()` 和执行器实现，检查 A 主机与 F 从机分支各 42 组相同归一化指令，包括中立、刹车、控制模式和饱和情况：中立指令一致，测试中的 PWM 指令最大差异为 1 μs，来自中位变化后的整数截断。

## 执行器指令准备

主循环按以下顺序执行：

```cpp
runControlCycle(); // 更新全局 commands
convertControlCommandsToPWM(commands);
prepareActuatorCommands();
applyAndTransmitActuatorCommands();
```

`runControlCycle()` 按主从身份选择并列流程，更新全局 `ControlCycleOutput commands`，仅保存本机和从机槽位的 `ActuatorCommand` 作动器贡献，待发送命令独立保存在 `slaveCommands[]`。`convertControlCommandsToPWM(commands)` 位于 `actuator_output.cpp`，直接按 `1500 + 本机 trim + 本机 rev × 1000 × 归一化指令` 生成本机物理 PWM，写入 `ActuatorOutput[5]`。

`prepareActuatorCommands(commands)` 直接打包从机逻辑控制输出，不计算控制律或交联。发送直接读取独立全局 `slaveCommands[]`，不建立 Slave 输出缓存或记录对应逻辑指令列。从机融合两路逻辑贡献后才转换成物理 PWM。

当前命令帧为 40 字节，传输控制模式与 int16 定点逻辑输出，与旧协议不兼容，所有节点需同步更新固件。各机的 `pwm_channel1_trim`、`pwm_channel2_trim`、`pwm_channel3_trim` 保存本机安装偏置，需在对应机体上调整并保存参数。命令接收和转发沿用原缓存逻辑，不增加命令超时策略。

A 机在主分配中完成本机交联和子机指令生成；B～G 在从分配中完成本机交联，再融合上级贡献。接收命令由通信对象持有 `SlaveCommand`，控制流程直接读取，不再使用 Local 中间标量。

油门机间字段传递 int16 定点逻辑控制输出；各机在本地应用 `PWM_CENTER_US`、油门 rev/trim 和最终输出保护。参数 `usb_throttle_debug` 保留为油门调试开关，诊断函数输出接收的逻辑输出和上级帧状态。

本地升降舵计算、副翼补偿和阻尼在控制/mixer 阶段完成，手动与前馈贡献在融合前明确归属。INDI 物理 PWM 标定输出在 mixer 中适配回逻辑方向，最终转换只恢复一次 rev/trim。`ActuatorOutput[2]` 表示最终本地升降舵 PWM；输出数组按副翼1、副翼2、升降舵、油门、方向舵排列，日志读取实际写出的值。

`pitch_controller` 为本机 SD 整数参数，0 选择 PID、1 选择 INDI，默认 1，下一周期生效，适用于增稳模式。手动不运行闭环；积分独立由本机 SD 整数参数 `integralControlEnabled` 控制（0 关闭、1 开启，默认 1），修改后下一周期生效。控制流程仅保留手动和增稳两种模式，通信编码为 0 手动、1 增稳，未增加失效切换或输出覆盖策略，具体边界见 [控制架构](control-architecture.md)。

## 执行器锁定与限幅

各机上电独立锁定。按键 36 长按 1 秒解锁并停止姿态屏幕刷新，再长按上锁、立即回到锁定位置并恢复刷新；短按继续将夹角清零。按键 31 的 IMU 标定会阻塞主循环，因此标定前也先上锁。锁定状态不经机间链路传播，每架飞机都需要单独操作。

锁定时五路输出直接为 `1500 + 本机该通道 trim`，不应用 rev、控制指令或解锁后的软件限幅。解锁后，有效控制输出按各路配置独立限幅（默认 1000～2000 μs）；NaN/Inf 控制结果回退到该通道的锁定位置。trim 的有限性和 -500～500 μs 范围由参数入口校验。Servo 驱动使用独立的 1000～2000 μs 范围。舵机绑定后立即输出锁定位置，随后每周期继续执行锁定判断。日志记录最终写入的五路值。

**油门锁定值也为 `1500 + pwm_channel4_trim`。若 `pwm_channel4_trim=0`，电调将收到 1500 μs；要让锁定油门为 1000 μs，必须先在该机 SD 参数文件中将 `pwm_channel4_trim` 设为 -500 并确认上电恢复。**当前默认仍为 0，不能直接带电调试运行此固件。

例如油门 `trim=-500`：上电和锁定时输出 1000 μs。若将第 4 路 `PWM_SERVO4_MIN_US` 改为 1100，则解锁后低于 1100 μs 的有效控制输出被抬到 1100 μs；重新锁定后回到 1000 μs。调整软件范围只需修改 `src/config/flight_config.h` 中对应通道的上下限。通道 1～5 依次为左副翼、右副翼、升降、油门、方向；每路下限须不大于上限，且软件范围必须包含于驱动范围内，否则编译会报错。

## 串口映射

串口宏统一定义在 `src/config/serial_config.h`，业务模块按设备用途调用；修改接线时只需调整此处映射。

| 名称 | 硬件串口 | 用途 |
| --- | --- | --- |
| `USBSerial` | `Serial` | USB 调试、参数读取与修改 |
| `ParentSerial` | `Serial6` | 上级机体链路 |
| `LeftChildSerial` | `Serial3` | 左侧子机链路 |
| `RightChildSerial` | `Serial5` | 右侧子机链路 |
| `ExternalImuSerial` | `Serial1` | 外置 IMU |
| `SbusSerial` | `Serial2` | SBUS 接收机 |
| `DsmSerial` | `Serial3` | DSM 接收机 |
| `StrainSensorSerial` | `Serial7` | 应变传感器 |
| `TelemetrySerial` | `Serial8` | 数传发送及 airdata 接收 |

## 构建

在仓库根目录执行：

```sh
pio run
```

清理构建产物：

```sh
pio run --target clean
```

连接 Teensy 并上传：

```sh
pio run --target upload
```

打开串口监视器：

```sh
pio device monitor
```

固件产物位于 `.pio/build/teensy41/`，其中 `firmware.hex` 用于烧录。`.pio/` 是生成目录，不应提交到 Git。

## 软件执行流程

`setup()` 完成串口、OLED、SD 卡、USB 参数服务、EEPROM、舵机、接收机、IMU 和空速传感器的初始化。当前 `initializeControlAllocation()` 调用已注释。随后 `loop()` 以目标 500 Hz 运行：

```text
传感器采样
  -> Madgwick 姿态估计
  -> 遥控/上级命令转换
  -> 手动或增稳控制
  -> INDI 与构型控制
  -> 当前构型的混控（动态分配调用已注释）
  -> PWM 输出与级联转发
  -> SD 日志、遥控归一化和循环限频
```

当前主要控制模式为：

- `MANUAL_MODE`：执行手动命令，构型 PID 清零。


- `STABILIZE_MODE`：串级姿态增稳，积分由参数 `integralControlEnabled` 控制，俯仰算法由 `pitch_controller` 选择。CH5 大于 0 时为手动，其余位置为增稳。

## 主要硬件接口

| 接口 | 当前用途 |
| --- | --- |
| Servo 1–5 / 引脚 2–6 | 左副翼、右副翼、升降、油门、方向 |
| SPI CS 10、37 | BMI088 加速度计、陀螺仪 |
| `Serial1` / 115200 | 外置 IMU 数据 |
| `Serial2` | SBUS 接收机 |
| `Serial3` / 921600 | 左侧级联链路 |
| `Serial5` / 921600 | 右侧级联链路 |
| `Serial6` / 921600 | 上级控制命令 |
| `Serial7` / 115200 | Modbus 应变传感器 |
| `Serial8` / 115200 | 数传/气动数据 |
| `Wire1` | SSD1306 OLED、MS4525 空速传感器、MT6701 夹角传感器（SDA=17、SCL=16） |
| 内置 SD | 飞行日志 `datalogNNN.txt` |

## 通信协议概览

- `55 60`：40 字节级联控制帧，包含当前模式、五路逻辑控制输出和俯仰目标。
- `55 71`：34 字节状态帧，包含左右 bool 标志、相对角、滚转角速度及滚转/俯仰/偏航三轴姿态；与旧 33 字节协议不兼容，各节点须同步更新。
- Modbus RTU：通过 `Serial7` 请求五通道应变数据，使用 CRC16/Modbus。
- `AA 55`：通过 `Serial8` 接收二进制气动数据包。

级联协议没有版本字段。修改帧长度、字段顺序或缩放系数时，必须同步更新所有节点固件。

## 开发注意事项

- `platformio.ini` 当前使用 `-w` 屏蔽全部编译警告，容易隐藏类型、返回值和变量遮蔽问题。
- `control_allocation.cpp` 的 `getQx()` 存在局部变量遮蔽全局 `Qx` 的风险，修改控制分配前应优先验证。
- Servo 6 和 Servo 7 都配置为引脚 9；旧电机引脚配置还与 BMI088 CS 37 存在复用。
- OLED、SD 和串口任务位于同步主循环，必须测量实际循环频率；动态分配的矩阵求逆与 SVD 当前未调用，重新启用时需评估执行时间。
- MT6701 使用 `Wire1` 的 7 位地址 `0x06`，不再与应变传感器共用 `Serial7`。
- 源文件中存在历史编码乱码；编辑时统一使用 UTF-8，并避免无意义地整体格式化大文件。

## 需求与验证

SD 飞行日志可使用 [Python 飞行数据浏览器](../tools/flight_data_browser/README.md) 读取，进行多机对比、子图分析、区段 FFT 和 CSV 导出。

功能、接口、性能和安全需求见 [需求说明](requirements.md)。提交控制代码前至少应执行一次完整 PlatformIO 构建；涉及真实硬件的变更还应完成拆桨地面测试和相应飞行试验记录。

所有飞机统一调用 `logFlightData()`，以 50 Hz 写入同一 CSV 格式。字段顺序为：时间（`current_time`，微秒；`dt`，秒）、本机传感器发布值与滤波值、八路归一化遥控指令、`ActuatorOutput[0..4]`、模式变量、`targets` 全部字段、各控制器输出（不记录 `pitchIndiControlOutput.diagnostics`）。列名直接使用代码中的完整变量/成员名和数组索引，不附加旧别名或单位后缀；浮点值保留六位小数，模式枚举和布尔值按整数记录。遥控输入列为 `radio_channel1_normalized`～`radio_channel8_normalized`，范围 [-1,1]；油门最低/失联值为 -1。不记录相邻机传感器状态，单机与编队不再分支选择表头。


## 统一惯性测量接口

`sensors/sensor_processing.h` 提供 `ang_vel[3]` 和 `ang_vel_filtered[3]`（deg/s，后者为传感器层统一滤波结果）、`ang_acc[3]`（deg/s²）和 `acc[3]`（g），数组按 X/Y/Z 排列。`sensors/filters.h` 提供 `att[3]`（deg，滚转/俯仰/偏航）和 `quat[4]`，按 W/X/Y/Z 排列，沿用现有姿态算法约定。内置 IMU 的 X 轴符号在传感器层统一到原控制和日志方向，外置 IMU 保持其安装轴映射。

主循环调用 `updateInertialSensors()`，按外置优先、否则当前 BMI088 路径进行采样和姿态估计，发布统一测量后计算角加速度。控制、日志、诊断、地面站和机间通信使用统一数组，具体型号的采样变量仅在 `sensors` 内部使用。

机间角速度为 `ang_vel_B[3]` 至 `ang_vel_G[3]`；现有协议仅传输滚转角速度，接收端只更新 `[0]`，`[1]`/`[2]` 保持 0，不能视为有效测量。协议包布局保持不变。SD 日志列名使用 `ang_vel[i]`、`ang_acc[i]`、`acc[i]` 和 `quat[i]`；所有飞机使用统一日志格式，仅记录本机选定数据源及统一滤波结果。

五路应变测量统一通过 `strain_value[5]`（`int32_t` 数组）读取，索引 0～4 对应传感器通道 1～5；保持原报文解析和数值单位。SD 日志中的姿态和应变列分别使用 `att[i]`、`strain_value[i]`。
