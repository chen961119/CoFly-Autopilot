# CoFly Autopilot 架构说明

## 项目定位与实现边界

CoFly Autopilot（Collaborative Distributed Autopilot，分布式协同飞控系统）面向多体飞行器、大展弦比柔性飞行器及编队。当前固件实现 A–G 多体级联节点的控制与通信，运行平台为 Teensy 4.1；通用编队控制仍属于扩展方向。

七机物理顺序为 `F-D-B-A-C-E-G`。A 主节点接收遥控命令，计算组合控制并向左右链路分发；从节点接收上级命令，应用本机控制、执行器方向与安装偏置，并逐级回传状态。身份与机数通过地面站参数配置，控制宏见 [flight_config.h](../src/config/flight_config.h)，串口映射见 [serial_config.h](../src/config/serial_config.h)。

七机分支采用构型相关的固定系数混控。`control_allocation.cpp` 保留五体动力学矩阵和普通 SVD 伪逆；`main.cpp` 中初始化与周期计算调用目前均已注释。该模块不等于已接入七机输出的动态分配器，也没有实现参考论文的构型 INDI / IWPI。本机俯仰 PID/INDI 由 SD 参数 `pitch_controller` 选择，主从并列的七层流程见 [控制架构](control-architecture.md)。

## 模块职责

| 模块 | 职责 |
| --- | --- |
| `src/main.cpp` | 初始化和周期任务编排 |
| `src/config/flight_config.h` | IMU、接收机与控制宏 |
| `src/config/aircraft_config.cpp` | 运行时节点身份与组合数量 |
| `src/system/flight_clock.cpp` | 时间更新和循环限频 |
| `src/sensors/sensor_processing.cpp`、`src/sensors/filters.cpp` | 传感器采样、校准、滤波与姿态估计 |
| `src/radio_comm/radio_comm.cpp` | 遥控接收接口 |
| `src/control/control_pipeline.cpp`、`src/control/control_modes.cpp` | 主从并列流程、原有三种模式与俯仰算法选择 |
| `src/control/control_modes.cpp`、`src/control/command_generation.cpp` | 主从模式与目标准备、从机命令生成 |
| `src/control/master_controller.cpp`、`src/control/slave_controller.cpp`、`src/control/*_controller.cpp` | 主/从模式分派及各独立控制器类 |
| `src/allocation/` | 主/从混控、交联与两路指令融合 |
| `src/parameters/control_parameters.cpp`、`src/parameters/control_parameters.h` | 可调控制参数与执行器方向/trim 的变量及默认值 |
| `src/parameters/parameter_registry.cpp` | 全部参数的名称、绑定、范围、存储位置及生效方式注册 |
| `src/control/control_allocation.cpp` | 保留的五体动力学与伪逆分配模型 |
| `src/actuators/actuator_output.cpp`、`src/system/flight_lock.cpp` | PWM 转换、本机输出准备、锁定和最终限幅 |
| `src/interaircraft_comm/interaircraft_comm.cpp` | 命令链 `AircraftCommandLink` 与状态链 `AircraftSensorLink` 的收发、解析及转发 |
| `src/interaircraft_comm/interaircraft_common.cpp` | 机间串口初始化与相对角查询 |
| `src/ground_station_service/ground_station_service.cpp` | USB 地面站设备服务 |
| `src/parameters/parameter_service.cpp` | 参数初始化、RAM 修改、保存调度、校准提交及 USB 参数协议 |
| `src/parameters/parameter_sd.cpp`、`src/parameters/parameter_eeprom.cpp` | SD 文件与 EEPROM 主备快照的保存、校验及恢复 |
| `src/parameters/parameter_record.cpp`、`src/parameters/parameter_text.cpp` | 两种介质共用的文本记录编解码与 CRC32 容器 |
| `src/logging/logger.cpp`、`src/telemetry/telemetry.cpp` | SD 日志与遥测 |
| `src/interface/human_interface.cpp`、`src/diagnostics/debug_print.cpp` | 按键、OLED 与诊断输出 |
| `src/diagnostics/firmware_debug.cpp` | 板上断点调试集成 |
| `src/control/control_modes.h/.cpp` | 模式准备、目标生成及跨模块模式状态 |
| `src/parameters/parameter_registry.h` | 各模块参数注册接口 |

各模块的头文件与实现统一存放于 `src/<模块>/`，跨模块引用使用相对 `src` 根目录的路径，例如 `control/control_modes.h`。项目不再建立单独的 `include` 目录。目录职责见[源码目录说明](../src/README.md)，参数模块的维护入口见[参数模块说明](../src/parameters/README.md)。

## 主循环与输出

`setup()` 建立通信链路，初始化显示、SD 与参数服务、传感器和执行器，随后初始化遥控接收、姿态、控制模式及滤波器。

```text
时间更新与传感器采样
  -> 姿态估计、空速与夹角
  -> 期望状态、按键与相邻节点状态
  -> 主机指令/归类/控制/分配，或从机控制/分配/融合
  -> PWM 转换、执行器命令准备
  -> 锁定/限幅、本机输出与级联发送
  -> SD 日志
  -> 遥控更新与归一化、USB 参数服务
  -> 500 Hz 节拍等待
```

遥控命令在周期末更新，供下一周期使用。500 Hz 是目标频率；OLED、SD、参数保存等同步操作的实际耗时仍需硬件测量，不能仅由 `loopRate(500)` 推断性能。

输出路径依次为 `runControlCycle()`、`convertControlCommandsToPWM(commands)`、`prepareActuatorCommands(commands)`、`applyAndTransmitActuatorCommands()`。交联在 mixer 中完成，融合后再应用本机中位、rev、trim。上级向从机发送 PWM 缩放前的 int16 定点逻辑控制输出。锁定时直接输出 `1500 + trim`，解锁后应用各通道独立的软件限幅（默认各路 1000～2000 μs）；非有限控制结果回退到锁定位置。Servo 驱动范围独立保持 1000～2000 μs。具体锁定油门行为见 [固件指南](firmware-guide.md#执行器锁定与限幅)。

## 兼容性与维护入口

项目名称、参数软件、USB 响应前缀 `@COFLY`、文本存储标识 `COFLY_PARAMS_V8` 及调试宏 `COFLY_TEENSY_DEBUG` 已统一使用 CoFly。参数存储仅支持当前共享文本 V8，不导入旧格式。仅有旧格式数据时，更新后通过地面站重新设置参数并完成校准。

修改身份使用地面站的 `aircraft_id` 参数，机数使用 `src/config/aircraft_config.h` 的编译宏 `COFLY_AIRCRAFT_COUNT`；参数名称、范围、存储介质和生效方式在 `src/parameters/parameter_registry.cpp` 调整。接线修改使用 `serial_config.h` 和具体传感器/执行器模块；控制律修改使用 `src/control/` 下对应的控制器类，混控与融合使用 `src/allocation/`；级联协议修改使用 `src/interaircraft_comm/` 中的收发实现。40 字节级联命令由 `AircraftCommandLink` 管理，按 left/right 分别实例化：A 使用左右两个独立对象，从机只使用所在侧对象接收和转发，每个对象绑定单一输出串口并独立保存命令缓存及发送时间。身份参数加载后绑定串口，固定槽位保持不变，接收控制模式及五路逻辑输出。对象流程及仍保留的状态回传接口见[机间通讯说明](../src/interaircraft_comm/README.md)。

需求与验收目标见 [软件需求说明](requirements.md)，参考论文及旧版代码对照见 [文档导航](README.md)。
