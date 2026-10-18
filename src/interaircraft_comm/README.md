# 机间通讯

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `interaircraft_comm.h` | 命令链类、状态链类、对象及公共结果接口 |
| `interaircraft_comm.cpp` | `AircraftCommandLink`、`AircraftSensorLink` 的收发实现 |
| `interaircraft_common.cpp` | 串口初始化、身份加载后的命令链绑定、本机翼尖相对角查询 |

## 修改规则

每当修改通讯中的AircraftCommandLink和AircraftSensorLink所传输的信息时，仅允许修改CommandPayload、setCommandLink、getCommandLink、printReceivedData、SensorPayload、getSensorLink、setSensorLink、void printReceivedData。

## 两串口接线与对象绑定

`beginInteraircraftLinks()` 固定初始化 Serial3（物理左侧）和 Serial5（物理右侧），均为 921600 baud；不再初始化 Serial6。`CommandLinks::left/right` 和 `SensorLinks::left/right` 仍表示 B/D/F、C/E/G 两个分支，不能直接理解为上级串口所在的物理方向。

| 身份 | 上级端口 | 下级端口 |
| --- | --- | --- |
| A | 无（nullptr） | 左 Serial3，右 Serial5 |
| B/D/F | Serial5 | Serial3 |
| C/E/G | Serial3 | Serial5 |

不按是否翼尖裁剪端口：翼尖同样绑定下级命令输出和状态输入，只是接口未接线。原 `interaircraft_comm.cpp` 的端口非空判断、首帧门控、收发与转发逻辑保持不变。上级端口只有命令解析器读取，下级端口只有状态解析器读取，不会竞争消费同一串口。

所有相邻节点按左接口 Serial3 ↔ 右接口 Serial5 接线，TX/RX 交叉、GND 共地；从机旧上级线需从 Serial6 迁移。各节点应统一更新固件及接线。单机和身份无效时仍不创建链对象。

2026-10-18 本地验证：普通与调试固件编译通过；使用实际身份配置、对象绑定及原收发实现进行主机模拟串口检查，覆盖 1/3/4/5/7 机构型，共 417 项断言通过，包括角色映射、翼尖保留下级端口、首帧门控、读取端口隔离、命令原帧转发、状态上行、有效期和 Serial6 无初始化/收发。临时测试文件已删除，尚未进行实板改线联调。

## 当前命令协议

命令帧 `55 60` 固定为 40 字节：帧头 2 字节、载荷 37 字节、累加校验 1 字节。载荷按小端紧密排列：`controlMode`（uint8，0 手动、1 增稳），15 个 int16 `controlOutputs`（逻辑值 ×4096）（每槽位为副翼1、副翼2、油门、方向舵、升降舵），3 个 int16 俯仰目标（0.001 rad）。

五路输出均为 PWM 缩放前的逻辑量，以 int16 定点编码（发送乘 4096，接收除 4096），1 对应 1000 μs，但报文不包含 PWM 中位、rev 或 trim，逻辑值不限制在 ±1，编码范围为 [-8, 7.999755859375]，超过编码范围时饱和限幅。手动模式的 elevator 是完整升降舵贡献，增稳模式是主机前馈，从机融合本机闭环贡献后才换算物理 PWM。上行状态包传输三轴姿态，不再回传升降舵 PWM。

副翼和升降舵的逻辑贡献均以下偏为正，包含发送、接收、融合后的全部 `ActuatorCommand.elevator`。俯仰角目标仍以抬头为正；编码/解码不额外取反。本次方向统一保持 `55 60` 帧头、40字节布局和比例不变，但旧上偏正固件的 elevator 语义不同，各节点应同步更新。

解码结果由命令对象持有 `SlaveCommand`，控制流程直接读取 `masterContribution` 和 `mode`，没有 Local 中间标量。左侧槽位对应 B/D/F，右侧对应 C/E/G；从机按链路编号提取对应槽位，转发整份缓存，不移位。发送限频仍为每链 100 Hz。接收函数恢复原有校验后缓存流程，半帧和坏校验不替换协议缓存；模式检查放在 `getCommandLink()` 内，非法模式不覆盖已解析的本机命令。发送在 `setCommandLink()` 更新缓存前拒绝 NaN/Inf 输入。初始模式为手动，油门逻辑量 -0.5。

协议修改仅放在载荷、设置、提取和打印函数中。低油门默认值由 `CommandPayload` 成员初始化提供，不使用自定义构造函数；接收、发送、配置和转发函数保持原实现。控制层直接调用 `getCommandLink()` 读取已解析的本机命令，不新增通信访问辅助函数。新协议与旧版不兼容，各节点需同步更新固件。

定点步长为 1/4096，四舍五入的最大误差为 1/8192（未触发饱和时，对应最终 PWM 约 0.122 μs）。计算和命令结构体仍使用 float32；int16 仅为线上编码，不包含 PWM 中位、rev 或 trim。级联节点直接转发原编码缓存，避免重复量化。命令帧由 70 字节缩短为 40 字节，相同波特率下传输时间减少约 42.9%；协议版本需在所有节点同步更新。


## 当前状态协议

`55 71` 状态帧仍为 34 字节（31 字节载荷）：左右 bool 标志及各 3 项 int16 相对滚转角、滚转角速度、滚转、俯仰、偏航。角度为 0.001 rad，角速度为 0.001 rad/s。原最后三项物理升降舵 PWM 已替换为偏航角，帧长虽相同，字段含义不兼容，各节点仍须同步更新固件。

接收端使用 `att_B[3]` 至 `att_G[3]`，索引 0/1/2 为滚转/俯仰/偏航，单位 rad，角速度为 rad/s；编解码直接乘/除 1000，不再经过度制。本机槽位写入 `att[3]`，外侧槽位汇总下游姿态，内侧槽位沿用原清零行为；左右槽位、上行频率、帧头和校验不变。删除子机升降舵 PWM 的全局缓存及日志列，执行器日志仅记录本机最终 `ActuatorOutput[5]`，不记录从机逻辑输出缓存。

`AircraftCommandLink::localAircraftLevel` 表示链路层级（A=1、B/C=2、D/E=3、F/G=4），不是唯一飞机 ID。`configure()` 的层级参数为 `ownAircraftLevel`，本机命令槽位按层级减 2 提取。该重命名不改变串口绑定、收发或转发行为。

## 角度与角速度编码

`CommandPayload::pitchAngles` 和 `SensorPayload` 的全部角度/角速度字段均保留 int16 类型。两种载荷各自声明静态 `ANGLE_SCALE=1000`，不占报文字节。

| 字段 | 发送 | 接收 | 单位/步长 |
| --- | --- | --- | --- |
| 命令 `pitchAngles` | rad ×1000，取最近整数 | 编码值 ÷1000 | rad / 0.001 rad |
| 状态 `relativeAngles/rollAngles/pitchAngles/yawAngles` | rad ×1000，取最近整数 | 编码值 ÷1000 | rad / 0.001 rad |
| 状态 `rollRates` | rad/s ×1000，取最近整数 | 编码值 ÷1000 | rad/s / 0.001 rad/s |

正负数均使用对称四舍五入，恰好半步时远离零；未饱和时理论量化误差不超过 0.0005 rad 或 0.0005 rad/s，另有 float32 舍入误差。int16 范围对应 [-32.768, 32.767] rad 或 rad/s（角速度约 [-1877.468, 1877.411] 度/秒）。命令目标超范围或为 NaN/Inf 时拒绝更新整个命令槽位；状态超范围时饱和，NaN/Inf 编码为 0，避免浮点转 int16 溢出；这些异常占位值本身不代表有效测量。

两个 `printReceivedData()` 均先将编码值除以 1000，再转度，角度标签为 `deg`、角速度标签为 `deg/s`，保留三位小数。例如编码 1000 表示 1 rad，打印 57.296 度；编码 -500 表示 -0.5 rad，打印 -28.648 度。`controlOutputs` 的 ×4096 编码与原始整数打印保持不变。

本次线上单位与之前的 0.1 度协议不兼容。帧头、大小、校验、槽位、收发与转发实现、100/200 Hz 频率及全部时间单位不变；各机需要统一使用新协议固件。独立数值/帧/打印验证见 [弧度协议验证报告](../../docs/interaircraft-radian-protocol.md)。
