# 3DR X6 数传与 CoFly 地面站

X6 使用透明串口传输 `GCS` / `PARAM` 文本协议。固件继续使用 Teensy 4.1 的 Serial8，项目串口预设为 **57600 / 8N1，无流控**，不是对所有 X6 出厂配置的假定。USB 直连仍使用 921600。

## 接线与供电

| X6 天空端信号 | Teensy 4.1 | 用途 |
| --- | --- | --- |
| TX | 34 / RX8 | 接收无线命令 |
| RX | 35 / TX8 | 发送回复 |
| GND | GND | 共地 |
| RTS、CTS（若有） | 不接 | 模块端也需关闭硬件流控 |
| VCC | 按实物手册接稳压电源 | 核实电压及峰值电流需求 |

按模块信号名称交叉连接 TX/RX，并核对实际插座针序，不依赖线色。Teensy UART 使用 3.3V 逻辑，输入不耐受 5V，连接前核实 X6 UART 电平。地面端通过 USB 连接电脑，天空端按模块规格供电。接好天线再上电。

目前未取得这对 X6 的准确硬件版本及专用手册，因此不指定插头针号、供电电压、电流或 USB 芯片驱动。一般 3DR/SiK 资料不能替代 X6 实物规格。[PJRC 串口表](https://www.pjrc.com/teensy/td_uart.html)提供 Serial8 针脚依据。

## 模块与地面站配置

1. 两端模块 UART 配置为 57600、8N1、关闭硬件流控。固件速率位于 `src/config/serial_config.h` 的 `TELEMETRY_BAUD`；修改天空端 UART 速率时也需修改固件并重新烧录，电脑串口速率与地面端 UART 匹配。
2. 两端使用匹配的无线配置，如频段、NETID、空中速率；UART 波特率与空中速率是不同参数。
3. 如果 X6 使用 SiK 固件，在独立配置工具中将两端 **MAVLINK 设为 0 / Raw Data**，关闭 RTS/CTS。项目传输自定义文本，需使用原始串口模式。参考 [SiK 配置说明](https://ardupilot.org/copter/docs/common-configuring-a-telemetry-radio-using-mission-planner.html)和 [SiK 高级配置](https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html)。非 SiK 版本按其手册选择透明模式，不能直接套用 SiK 的 AT 寄存器编号。
4. 退出模块配置工具，启动 `tools/ground_station/start.cmd` 或 `dist/CoFlyGroundStation.exe`，选择 **3DR X6**、地面端 COM 口和 **57600** 后连接。也可在应用设置开启自动连接：同时检测 USB / X6，均可用时优先 USB；连接期间保持当前链路，请求超时仅提示，不主动断开或重新扫描；串口报错或关闭后重新选择。详细规则见 [地面站自动连接](../tools/ground_station/README.md#自动连接)。缺少 COM 口时按实际 USB 芯片安装对应驱动。
5. 先识别固件，确认 CAPS 回复，再检查实时姿态、构型及参数读取。查看完整构型需连接 A 主机。USB 与 3DR X6 分别保存串口波特率。

## 更高波特率

57600 是项目预设，并非已确认的 X6 硬件上限。地面站已提供 115200、230400 等波特率选项；飞控通过 `TELEMETRY_BAUD` 配置 UART。上游 [SiK 串口实现](https://github.com/ArduPilot/SiK/blob/master/Firmware/radio/serial.c)的支持表包含 115200 和 230400，但尚未确认这对 X6 的厂家、硬件版本或固件，不能据此保证实物支持这些速率。

确认模块支持后，可先以 115200 做地面验证：将天空端模块 UART 与飞控 `TELEMETRY_BAUD` 匹配并重新烧录飞控，将地面端模块 UART 与地面站波特率匹配；自动连接使用已保存的数传波特率。保持 8N1、透明模式、无硬件流控，检查 CAPS/DATA/CONFIG/PARAM 往返及持续收发稳定性。仅修改地面站或飞控一处会使对应 UART 无法正常通信。

UART 速率与无线空中速率分开配置。根据 [SiK 高级配置说明](https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html)，空中速率、纠错及无线开销会限制实际吞吐；单独提高 UART 波特率不保证改善断联，透明模式下也不能假定有 MAVLink 自适应流量控制。未确认模块能力前，项目默认值继续保持 57600。

## 链路行为

`beginTelemetryLink()` 初始化 Serial8 并增加 1024 字节收发缓冲；`telemetry()` 轮询参数及地面站服务。USB 与无线分别维护解析、回复及参数列表状态，每周期每链路最多收发各 64 字节，按可写空间分片发送。没有请求时不主动广播数据，也不发送模块 AT 配置命令。

地面站最高每 200ms 发起一次数据请求，飞行页交替读取 DATA/CONFIG；实际刷新率取决于无线链路。参数列表使用八秒无有效进展超时，允许总读取时间超过八秒。锁定保护、保存与重启规则沿用原实现。旧 `AA 55` 气动输入与 `55 71` 二进制遥测保持停用。

软件构建及界面检查不能替代 X6 实物验证。当前未烧录设备、配置电台或验证射频链路；实物需确认供电、逻辑电平、配对、UART 设置及 CAPS/DATA/CONFIG/PARAM 往返。

2026-10-18 本地验证：`teensy41` 与 `teensy41_debug` 均编译通过；四组实际 Tk 界面检查覆盖 USB/X6 默认值、X6 自定义端口及波特率保留、无效连接类型回退，并验证预设切换与设置保存。地面站 EXE 已重新打包，`--self-test` 及设置输出检查通过。检查通过标准输入运行，临时设置目录已自动删除，无遗留测试脚本。
