# CoFly Autopilot 系统架构

## 定位与实现边界

CoFly Autopilot（Collaborative Distributed Autopilot，分布式协同飞控系统）面向多体飞行器、大展弦比柔性飞行器及编队。当前实现基于 Teensy 4.1 的 A～G 多体级联控制；通用编队控制属于扩展方向。

七机物理顺序为 `F-D-B-A-C-E-G`。A 接收遥控输入，计算组合控制并向左右链路分发；从节点执行本机控制、安装映射和状态回传。`aircraft_id` 是重启生效的身份参数，数量由 `COFLY_AIRCRAFT_COUNT` 编译宏决定，支持 1/3/4/5/7；同一数量及硬件配置的各节点共用固件。首次设置见 [参数与持久存储](parameter-storage.md#第一次使用)。

当前多机混控采用构型相关的固定系数。`src/control/control_allocation.cpp` 保留五体动力学矩阵和普通 SVD 伪逆，初始化与周期计算调用均已注释，未接入七机输出，也未实现参考论文的构型 INDI / IWPI。研究背景见 [研究方法与控制演进](archive/CONTROL_ARCHITECTURE_ANALYSIS.md)。

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

## 初始化与主循环

`setup()` 建立串口和通信链路，初始化显示、SD、参数服务、传感器和执行器，再初始化遥控、姿态、模式与滤波器。

```text
时间更新、传感器采样、姿态估计、空速与夹角
  → 按键和相邻节点状态
  → runControlCycle()：模式/目标 → 主或从控制 → 混控/融合
  → convertControlCommandsToPWM()
  → prepareActuatorCommands()
  → applyAndTransmitActuatorCommands()：锁定/限幅、输出与级联发送
  → SD 日志
  → 遥控更新与归一化、USB 参数服务
  → 500 Hz 节拍等待
```

遥控输入在周期末更新，供下一周期使用。500 Hz 是目标频率；OLED、SD 和参数保存等同步操作的最坏耗时需要硬件测量，不能由限频函数推断。

## 兼容性边界

- 参数仅支持共享文本 `COFLY_PARAMS_V8`，USB 响应前缀为 `@COFLY`。旧格式恢复规则见 [存储迁移](parameter-storage.md#格式与介质迁移)。
- 当前级联命令帧为 40 字节，状态帧为 34 字节；所有节点需使用兼容的布局、缩放和舵面方向语义，详见 [机间通讯模块](../src/interaircraft_comm/README.md)。
- 执行器锁定、最终限幅和升级时的升降舵方向参数调整统一在 [固件指南](firmware-guide.md) 维护。
- 目标需求与尚待闭环的问题见 [软件需求](requirements.md)；需求条目和历史检查均不等同于当前实现已通过验收。
