# CoFly Autopilot 固件指南

本文负责当前 Teensy 4.1 固件的配置、接线与运行操作。控制逻辑见 [控制架构](control-architecture.md)，首次身份设置和存储机制见 [参数与持久存储](parameter-storage.md)。

## 配置与构建

| 配置 | 当前入口与默认值 |
| --- | --- |
| 飞机身份 | EEPROM 参数 `aircraft_id`，0 未配置，1=A～7=G，重启生效 |
| 组合数量 | `src/config/aircraft_config.h` 中 `COFLY_AIRCRAFT_COUNT`，默认 3，支持 1/3/4/5/7 |
| 单机/多机 | 数量 1 为单机，身份设为 A；`TEAM` 只保留为存储配置标识 |
| IMU 与接收机 | `src/config/flight_config.h` 当前定义 `INTIMU`、`USE_SBUS_RX` |
| 俯仰与积分 | 本机 SD 参数 `pitch_controller=1`（INDI）、`integralControlEnabled=1` |
| 特定机型宏 | `expensive` 当前注释停用，硬件配置以 `flight_config.h` 为准 |

同一数量和硬件配置的 A～G 共用固件，逐机设置身份并重启。改变数量需重新构建；已有参数不会因数量改变自动换成另一套增益。控制模式与策略入口见 [控制架构](control-architecture.md#模式策略和状态)。

开发平台为 Arduino/Teensyduino、NXP i.MX RT1062（600 MHz），使用 PlatformIO 的 `teensy41` 环境。依赖由 [platformio.ini](../platformio.ini) 和 `lib/` 管理。在仓库根目录执行：

```sh
pio run -e teensy41
pio run -e teensy41 -t upload
pio device monitor -e teensy41
```

固件位于 `.pio/build/teensy41/firmware.hex`，上传协议为 `teensy-gui`，监视器默认 921600 baud。需要完整清理时执行 `pio run -e teensy41 -t clean` 后重新构建。`.pio/`、本机 IDE 配置和打包产物不提交。

板上断点调试使用 `teensy41_debug` 和 VS Code 的“TeensyDebug：编译、上传并调试”。调试固件的 `Serial` 保留地面站，`SerialUSB1` 用于 GDB；断点暂停时地面站请求也暂停。克隆后先构建以生成本机配置，详见 [板上调试](../tools/teensy_debug/README.md)。

## USB / X6 地面站

[地面站软件](../tools/ground_station/README.md) 位于 `tools/ground_station/`，支持参数读取、就地编辑、SD 恢复、重启提示、传感器/遥控/飞行数据显示及串口助手。参数页双击当前值编辑，Enter 保存，Esc 取消。启动方式、协议和电脑端依赖统一在工具文档维护。

USB 参数与地面站协议共用 `Serial`，3DR X6 通过 `Serial8` 独立收发同一协议。两条链路各自维护解析和回复状态；自动连接优先 USB，规则见地面站工具文档，接线和模块配置见 [X6 数传接入](x6-radio.md)。立即生效参数接受后在 RAM 中使用，存储采用 2 秒合并保存；身份重启生效。传感器校准与设置的部分 USB 接口仍返回 `NOT_IMPLEMENTED`，不能据此代替板上按键校准。

## 执行器映射与方向迁移

`PWM_CENTER_US=1500`；本机五路通道依次为左副翼、右副翼、升降舵、油门、方向舵。安装参数为 `pwm_channel1_rev/trim`～`pwm_channel5_rev/trim`，trim 编译默认均为 0 μs，保存到各自 SD 卡。有效逻辑指令转换为：

```text
PWM_i = 1500 + trim_i + rev_i * 1000 * command_i
```

trim 不随 rev 反向。副翼和升降舵逻辑量均以下偏为正；控制/交联/融合完成后才应用安装映射，公式在 [控制架构](control-architecture.md#逻辑坐标交联与融合) 维护。油门也以 1500 为转换基准，沿用现有逻辑尺度；不能把逻辑值直接当作物理推力。

升降舵 `pwm_channel3_rev` 默认为 1。**同一安装从旧上偏正语义升级时，应把旧保存的 rev3 取反后保存，例如 -1 改为 1；trim 保持原值。** 已保存值会覆盖源码默认值，没有自动方向迁移标记。全链路节点须同步采用下偏正语义，即使命令帧长度未变也不能混用。

INDI 以通道3配平中位为零舵角，内部使用浮点物理 PWM，出口为统一逻辑尺度；计算、单位和模型限制见 [俯仰 INDI](indi-variable-naming.md)。

## 执行器锁定与限幅

各机上电独立锁定。按键 36 长按 1 秒解锁并停止姿态屏幕刷新，再长按上锁并恢复刷新；短按清零夹角。按键 31 的 IMU 标定会阻塞主循环，因此标定前先上锁。锁定状态不经机间链路传播，每架飞机需要独立操作。身份未配置、IMU 校准无效或存在待重启设置时禁止解锁。

| 情况 | 本机输出 |
| --- | --- |
| 上电/锁定 | 直接输出 `1500 + 本机通道 trim`，不应用 rev、控制指令或解锁软件限幅 |
| 解锁且控制值有效 | 按本通道软件上下限限幅后输出 |
| 控制值 NaN/Inf | 回退到本通道锁定位置 |
| 驱动范围 | Servo attach 独立限制为 1000～2000 μs |

**默认油门 trim=0 时，锁定油门也是 1500 μs。若电调安全输入要求 1000 μs，应先将本机 `pwm_channel4_trim=-500` 保存并确认上电恢复；首次上电、固件变更和身份切换均须拆桨核对实际输出。**

五路软件上下限为 `pwm_servo1_min_us/max_us`～`pwm_servo5_min_us/max_us`，默认均为 1000～2000 μs。锁定时修改，接受后立即生效并保存到本机 SD；每路须在驱动范围内满足 min≤max，相等允许。完整参数规则和异常恢复见 [参数声明](parameter-storage.md#参数声明)。

锁定输出不受软件限幅影响。例如油门 trim=-500、软件下限=1100：锁定输出 1000 μs，解锁后有效输出最低 1100 μs，重新锁定回到 1000 μs。日志中的 `ActuatorOutput[0..4]` 保存锁定/最终限幅后的软件写出值。

## 主要硬件接口

| 接口 | 用途 |
| --- | --- |
| Servo 1～5 / 引脚 2～6 | 左副翼、右副翼、升降、油门、方向 |
| SPI CS 10、37 | BMI088 加速度计、陀螺仪 |
| `Wire1`，SDA=17、SCL=16 | OLED、MS4525 空速、MT6701 夹角 |
| 内置 SD | `datalogNNN.txt`、参数主备文件 |

串口宏集中在 `src/config/serial_config.h`：

| 宏 | 硬件串口 | 用途 |
| --- | --- | --- |
| `USBSerial` | `Serial` | 地面站、参数与调试 |
| `ParentSerial` | `Serial6` | 上级链路，921600 baud |
| `LeftChildSerial` | `Serial3` | 左侧链路，921600 baud |
| `RightChildSerial` | `Serial5` | 右侧链路，921600 baud |
| `ExternalImuSerial` | `Serial1` | 外置 IMU，115200 baud |
| `SbusSerial` | `Serial2` | SBUS |
| `DsmSerial` | `Serial3` | DSM 备选，与左侧链路复用需核对 |
| `StrainSensorSerial` | `Serial7` | Modbus 应变，115200 baud |
| `TelemetrySerial` | `Serial8` | 3DR X6 双向地面站，57600 / 8N1，无流控（TX=35、RX=34） |

预留 Servo 6/7 同为引脚 9，旧电机候选引脚含 BMI088 CS 37，当前均为注释代码；重新启用前须处理冲突。

## 传感器与统一测量接口

MT6701 位于 `Wire1` 的 7 位地址 `0x06`，先读 `0x03` 再读 `0x04`，14 位数据换算为 0～360°。`relativeAngle_ready` 减零点后转换到 ±180°，A/B/D/F 反向、C/E/G 正向。按键清零通过参数服务保存 `mt6701_zero`。读失败保留最后有效值，每 20 ms 重试，禁止保存错误零点。

MT6701 仅用于测量与日志。构型控制、分配、OLED、地面站及机间构型使用相邻 IMU 滚转差；OLED 无有效下游姿态时显示 `--`。完整构型显示需连接 A 主机。

`updateInertialSensors()` 按外置优先、否则内置路径采样和姿态估计，发布以下统一接口；具体采样与滤波配置以 `sensor_processing.cpp` 和 `filters.cpp` 为准。

| 接口 | 单位与顺序 |
| --- | --- |
| `ang_vel[3]`、`ang_vel_filtered[3]` | deg/s，X/Y/Z，后者为传感器层统一滤波值 |
| `ang_acc[3]` | deg/s²，X/Y/Z |
| `acc[3]` | g，X/Y/Z |
| `att[3]` | deg，滚转/俯仰/偏航 |
| `quat[4]` | W/X/Y/Z，沿用姿态算法约定 |
| `strain_value[5]` | int32，应变通道 1～5，保持原数值单位 |

内置路径在传感器层统一 X/Z 角速度符号，外置路径保持安装轴映射。`ang_vel_B[3]`～`ang_vel_G[3]` 目前仅通过状态协议更新滚转分量 [0]，[1]/[2] 为 0，不能作为有效测量。

## 通信与日志

| 协议 | 帧头 | 当前内容 |
| --- | --- | --- |
| 级联命令 | `55 60` | 40 字节，模式、五路定点逻辑输出和俯仰目标 |
| 状态汇总 | `55 71` | 34 字节，左右标志、相对角、滚转角速度和三轴姿态 |
| 应变 | Modbus RTU | 五通道请求，CRC16/Modbus |
| 地面站 | `GCS` / `PARAM` | USB / Serial8 ASCII 行协议 |

Serial8 旧 `AA 55` 气动读取和 `55 71` 二进制遥测已停用，避免吞掉地面站命令或混入文本回复；外部气动数据需另行分配串口。

级联协议没有版本字段。改变布局、长度、缩放或方向语义须同步更新所有节点；当前命令/状态帧与旧 50/33 字节版本不兼容。对象、缓存与路由详见 [机间通讯模块](../src/interaircraft_comm/README.md)。

所有飞机调用 `logFlightData()`，以 50 Hz 写入同一 CSV 格式。字段按时间、本机传感器/滤波值、八路归一化遥控、五路最终执行器 PWM、模式、`targets` 和各控制器输出排列。列名使用代码中的完整成员名和索引，浮点保留六位小数，枚举/布尔按整数记录；不记录相邻机传感器、从机逻辑输出或 INDI 中间 PWM。

遥控列 `radio_channel1_normalized`～`radio_channel8_normalized` 为 [-1,1]，油门最低/失联值为 -1；控制目标 throttle 转为 0～1。INDI 字段单位见 [INDI 变量约定](indi-variable-naming.md#变量与发布接口)。日志读取、FFT 与导出见 [飞行数据浏览器](../tools/flight_data_browser/README.md)。

## 开发与验收

主循环、模块职责和动态分配的停用状态见 [系统架构](architecture.md)，无需在此重复控制流程。OLED、SD、串口和参数写入均可能影响周期，需在目标硬件测量 500 Hz 时序。`platformio.ini` 当前使用 `-w`、`-ffast-math`，相关验收要求及未闭环问题见 [软件需求](requirements.md)。

修改控制逻辑后完成完整构建，硬件行为变更还需拆桨地面测试及相应飞行记录。历史数值报告及其产物已清理，不代表当前工作区已重新构建或完成板上验证。
