# 机间通讯

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `interaircraft_comm.h` | 命令链类、状态链类、对象及公共结果接口 |
| `interaircraft_comm.cpp` | `AircraftCommandLink`、`AircraftSensorLink` 的收发实现 |
| `interaircraft_common.cpp` | 串口初始化、身份加载后的命令链绑定、本机翼尖相对角查询 |

## 修改规则

每当修改通讯中的AircraftCommandLink和AircraftSensorLink所传输的信息时，一般仅允许修改CommandPayload、setCommandLink、getCommandLink、printReceivedData、SensorPayload、getSensorLink、setSensorLink、void printReceivedData。

## 当前命令协议

命令帧 `55 60` 固定为 40 字节：帧头 2 字节、载荷 37 字节、累加校验 1 字节。载荷按小端紧密排列：`controlMode`（uint8，0 手动、1 无积分增稳、2 有积分增稳），15 个 int16 `controlOutputs`（逻辑值 ×4096）（每槽位为副翼1、副翼2、油门、方向舵、升降舵），3 个 int16 俯仰目标（0.1 度）。

五路输出均为 PWM 缩放前的逻辑量，以 int16 定点编码（发送乘 4096，接收除 4096），1 对应 1000 μs，但报文不包含 PWM 中位、rev 或 trim，逻辑值不限制在 ±1，编码范围为 [-8, 7.999755859375]，超过编码范围时饱和限幅。手动模式的 elevator 是完整升降舵贡献，增稳模式是主机前馈，从机融合本机闭环贡献后才换算物理 PWM。上行状态包传输三轴姿态，不再回传升降舵 PWM。

解码结果由命令对象持有 `SlaveCommand`，控制流程直接读取 `masterContribution` 和 `mode`，没有 Local 中间标量。左侧槽位对应 B/D/F，右侧对应 C/E/G；从机按链路编号提取对应槽位，转发整份缓存，不移位。发送限频仍为每链 100 Hz。接收函数恢复原有校验后缓存流程，半帧和坏校验不替换协议缓存；模式检查放在 `getCommandLink()` 内，非法模式不覆盖已解析的本机命令。发送在 `setCommandLink()` 更新缓存前拒绝 NaN/Inf 输入。初始模式为手动，油门逻辑量 -0.5。

协议修改仅放在载荷、设置、提取和打印函数中。低油门默认值由 `CommandPayload` 成员初始化提供，不使用自定义构造函数；接收、发送、配置和转发函数保持原实现。控制层直接调用 `getCommandLink()` 读取已解析的本机命令，不新增通信访问辅助函数。新协议与旧版不兼容，各节点需同步更新固件。

定点步长为 1/4096，四舍五入的最大误差为 1/8192（未触发饱和时，对应最终 PWM 约 0.122 μs）。计算和命令结构体仍使用 float32；int16 仅为线上编码，不包含 PWM 中位、rev 或 trim。级联节点直接转发原编码缓存，避免重复量化。命令帧由 70 字节缩短为 40 字节，相同波特率下传输时间减少约 42.9%；协议版本需在所有节点同步更新。


## 当前状态协议

`55 71` 状态帧仍为 34 字节（31 字节载荷）：左右 bool 标志及各 3 项 int16 相对滚转角、滚转角速度、滚转、俯仰、偏航。角度为 0.1 度，角速度为 0.1 度/秒。原最后三项物理升降舵 PWM 已替换为偏航角，帧长虽相同，字段含义不兼容，各节点仍须同步更新固件。

接收端使用 `att_B[3]` 至 `att_G[3]`，索引 0/1/2 为滚转/俯仰/偏航，单位 deg。本机槽位写入 `att[3]`，外侧槽位汇总下游姿态，内侧槽位沿用原清零行为；左右槽位、上行频率、帧头和校验不变。删除子机升降舵 PWM 的全局缓存及日志列，执行器日志仅记录本机最终 `ActuatorOutput[5]`，不记录从机逻辑输出缓存。

`AircraftCommandLink::localAircraftLevel` 表示链路层级（A=1、B/C=2、D/E=3、F/G=4），不是唯一飞机 ID。`configure()` 的层级参数为 `ownAircraftLevel`，本机命令槽位按层级减 2 提取。该重命名不改变串口绑定、收发或转发行为。
