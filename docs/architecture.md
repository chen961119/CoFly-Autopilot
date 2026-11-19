# CoFly Autopilot 系统架构

## 定位与实现边界

CoFly Autopilot（Collaborative Distributed Autopilot，分布式协同飞控系统）面向多体飞行器、大展弦比柔性飞行器及编队。当前实现基于 Teensy 4.1 的 A～G 多体级联控制；通用编队控制属于扩展方向。

七机物理顺序为 `F-D-B-A-C-E-G`。A 接收遥控输入，计算组合控制并向左右链路分发；从节点执行本机控制、安装映射和状态回传。`aircraft_id` 是重启生效的身份参数，数量由 `COFLY_AIRCRAFT_COUNT` 编译宏决定，支持 1/3/4/5/7；同一数量及硬件配置的各节点共用固件。首次设置见 [参数与持久存储](parameter-storage.md#第一次使用)。

当前多机混控采用构型相关的固定系数。旧五体动态分配模型的数学结构和论文方法见 [研究方法与控制演进](archive/CONTROL_ARCHITECTURE_ANALYSIS.md)。

## 模块与修改入口

| 模块 | 职责与主要入口 |
| --- | --- |
| `src/main.cpp`、`src/system/flight_clock.cpp` | 初始化、周期任务编排、计时和循环限频 |
| `src/config/` | `flight_config.h` 定义硬件配置，`aircraft_config.h/.cpp` 定义数量和运行身份，`serial_config.h` 定义串口映射 |
| `src/sensors/` | 采样、校准、滤波、姿态估计和统一测量接口 |
| `src/radio_comm/` | 遥控接收和通道归一化 |
| `src/control/` | 模式准备、主从目标、独立控制器及控制周期编排 |
| `src/allocation/` | 主/从混控、交联和指令融合 |
| `src/actuators/`、`src/system/flight_lock.cpp` | PWM 转换、锁定、限幅与硬件输出 |
| `src/interaircraft_comm/` | 命令链 `AircraftCommandLink`、状态链 `AircraftSensorLink` 的收发与转发 |
| `src/parameters/` | 参数声明、注册、编解码、SD/EEPROM 存储与 USB 参数服务 |
| `src/ground_station_service/` | USB 地面站设备服务 |
| `src/logging/`、`src/telemetry/` | SD 日志与遥测 |
| `src/interface/`、`src/diagnostics/` | 按键、OLED、诊断输出与板上调试 |

头文件与实现同置于 `src/<模块>/`，跨模块引用相对 `src` 根目录，例如 `control/control_modes.h`。详细文件职责见 [源码目录](../src/README.md)。控制接口见 [控制架构](control-architecture.md)，参数注册与格式见 [参数与持久存储](parameter-storage.md)。

## FreeRTOS 任务与数据流

Teensy 4.1 的 i.MX RT1062 使用单核 Cortex-M7。工程采用 FreeRTOS 11.2.0 的 Teensy 板级移植 v4，并锁定配套 Arduino core。NXP SDK 提供同芯片评估板的官方 FreeRTOS 支持。Teensy 移植由社区维护，上游将它标为实验性。[PJRC 板卡规格](https://www.pjrc.com/store/teensy41.html) · [NXP SDK](https://mcuxpresso.nxp.com/mcuxsdk/latest/boards/RT/evkbmimxrt1060/index.html) · [板级移植及限制](https://github.com/tsandmann/freertos-teensy)

`setup()` 创建一次性任务 `Startup` 并启动调度器。`Startup` 恢复参数、建立机间链路、挂接舵机并写入本机锁定 trim，随后完成 IMU、模式与滤波器初始化。设备就绪后，`Startup` 在暂停调度期间创建下表中的五个业务任务，恢复调度后删除自身。五个业务任务各自拥有静态分配于片上 OCRAM 的栈，按各自的优先级运行。另有内核和板级后台任务。

| 应用任务 | 优先级 | 节拍及外设所有权 |
| --- | ---: | --- |
| Control | 5 | 每 2 个 1 ms tick 读取本机 BMI088 或外置 IMU，完成姿态估计、控制和 PWM 写出 |
| Links | 4 | 每 1 ms 服务接收机和 Serial3/5。按原协议下发 100 Hz 命令，上行 200 Hz 状态 |
| Aux | 3 | 每 2 ms 服务 MT6701、按键和应变串口。空速目标 50 Hz，应变请求 10 Hz。锁定时 OLED 目标 10 Hz，每次仅写 16 字节图像。启用 BMP280 时每 500 ms 读取 |
| Parameters | 2 | 轮询 USB / X6 会话、发布参数、处理标定结果、发送诊断以及协调重启 |
| Storage | 1 | 有请求时处理 SD/EEPROM、文件传输或 50 Hz CSV 日志。后台调用者等待存储结果 |

主控在周期开始锁存辅助测量和机间输入，直接读取本机 IMU 并计算本周期输出。写 PWM 后发布机间发送值和同周期服务快照，再锁存供下一周期使用的遥控值。过期节拍跳过，沿用原有锁定条件。首次控制计算的 `dt` 为 2 ms，后续使用实测间隔。无效身份和待重启状态继续提供参数与诊断服务，PWM 回到锁定位置。

参数任务校验整组变更后在短临界区提交。日志缓冲能容纳 8 个周期样本，写满时主控记录丢样数并继续。USB 文件会话开始时写完已接受的样本并刷盘，会话内暂停新日志及 SD 参数保存。原有两个 IMU 标定入口都保留。按键标定在锁定 PWM 得到确认后，每个主控周期最多采 8 个原始样本，组内间隔 100 µs，共 12000 个。地面站标定仍使用 2000 个样本及水平、静止判据。

## 兼容性边界

- 参数仅支持共享文本 `COFLY_PARAMS_V8`，USB 响应前缀为 `@COFLY`。旧格式恢复规则见 [存储迁移](parameter-storage.md#格式与介质迁移)。
- 当前级联命令帧为 40 字节，状态帧为 34 字节；所有节点需使用兼容的布局、缩放和舵面方向语义，详见 [机间通讯模块](../src/interaircraft_comm/README.md)。
- 执行器锁定、最终限幅和升级时的升降舵方向参数调整统一在 [固件指南](firmware-guide.md) 维护。
- 目标需求与尚待闭环的问题见 [软件需求](requirements.md)；需求条目和历史检查均不等同于当前实现已通过验收。
