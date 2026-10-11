# 机间通讯

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `interaircraft_comm.h` | 命令链类、状态链类、对象及公共结果接口 |
| `interaircraft_comm.cpp` | `AircraftCommandLink`、`AircraftSensorLink` 的收发实现 |
| `interaircraft_common.cpp` | 串口初始化、身份加载后的命令链绑定、本机翼尖相对角查询 |

## 命令链对象

`AircraftCommandLink` 与状态链一样按侧实例化：A 初始化静态对象 left、right；B/D/F 只使用 left，C/E/G 只使用 right。`CommandLinks::left`、`CommandLinks::right` 指针记录启用的对象，单机及未配置身份时均为空。每个对象只有一个下游输出串口、一个可选上级输入、一份协议结构体及接收缓存、一个发送限频时间，不使用堆分配或虚函数。静态存储仍为两侧对象预留 RAM。

`initializeCommandLinks()` 在参数身份加载后配置对象。A 的 left/right 分别绑定 LeftChildSerial/RightChildSerial；从机绑定所在侧下游输出及 ParentSerial 输入。链路编号 A=1、B/C=2、D/E=3、F/G=4，提取命令时减 2 转为槽位 0～2。

主机流程（执行器输出完成后，两侧对象各自以 100 Hz 限频）：

```cpp
prepareChildCommandBuffer(true);
CommandLinks::left->sendPreparedChildCommands();
prepareChildCommandBuffer(false);
CommandLinks::right->sendPreparedChildCommands();
```

左右缓存相互独立，不再覆盖复用同一对象。单侧 prepareChildCommandBuffer() 将三机命令、灯光和模式标志写入所在侧对象。setCommandLink() 检查索引 0～2，统一完成输入到协议类型的转换；模式标志整帧共用，三次设置传入相同值。

从机调用 receiveCommandData()、forwardReceivedChildCommands()，由统一编排函数选择所在侧对象。有效完整帧接收后 getCommandLink() 提取本机输入；转发保留完整上级协议字段，不清零、不移位、不覆盖模式标志。本机和发送缓存的默认低油门偏移仍为 1000 - PWM_CENTER_US，收到首帧前禁止转发。hasReceivedCommandData() 供诊断查询所在侧链路。

## 协议与转发

命令仍使用原有 50 字节帧：2 字节帧头、积分控制与手动标志各 1 字节、12 个 int16 舵机偏移、3 个 int16 俯仰角、3 个 int16 手动升降舵偏移、3 个 int16 前馈偏移、3 字节灯光、1 字节累加校验和。角度以 0.1 度编码，整数为小端有符号值。FRAME_SIZE = HEADER_SIZE + sizeof(CommandPayload) + CHECKSUM_SIZE；收发统一通过 memcpy 复制协议结构体，不复制类对象。修改协议时更新 CommandPayload 结构体、setCommandLink()、getCommandLink()、printReceivedData() 和新增字段的业务使用，各机必须使用同一份协议定义。帧长度随结构体大小自动计算，打印字段手动维护。

左链固定槽位为 B/D/F，右链为 C/E/G。接收先收齐字节并校验，再更新同一对象的命令属性和本机控制输入；半帧、坏帧不会覆盖有效命令。转发保留整份协议结构体，不清零、不移动或压缩数组，不再对俯仰角进行浮点解码后重新量化。重复转发幂等，灯光随当前有效命令传递。

`Local_*` 是本机控制接口，在接收时独立提取；转发复用协议结构体，本机命令在接收时独立提取。属性保存的是有符号控制偏移（μs），不是包含 rev/trim 的最终物理 PWM。

## 状态回传

每个 `AircraftSensorLink` 对象只管理一个子机输入串口，持有一份接收缓存、接收位置、有效标志、接收时间和协议结构体；属性不区分左右。对象直接命名为 `left`、`right`：A 在初始化时构造并配置两个对象；B/D/F 只构造并配置 left，C/E/G 只构造并配置 right。两者分别对接 LeftChildSerial、RightChildSerial，内部静态指针只记录启用的对象，不再使用主对象和 additional 的区分。静态存储不使用堆分配，但因飞机身份为启动参数，两份存储空间仍在固件 RAM 中预留。

`SensorPayload` 采用 pack(1) 紧密排列：先一个 bool `isLeft`（true 左链 B/D/F，false 右链 C/E/G），再 relativeAngles、rollRates、rollAngles、pitchAngles、elevatorPwm 五组 int16_t[3]。前四组单位分别为 0.1 度、0.1 度/秒、0.1 度、0.1 度；PWM 单位为 μs。FRAME_SIZE = HEADER_SIZE + sizeof(SensorPayload) + CHECKSUM_SIZE，目前 34 字节，帧头仍为 55 71。该协议与旧 33 字节状态帧不兼容，所有节点须同步更新。

与命令链统一：`setSensorLink()` 汇总本机与下游、计算相邻 IMU 滚转角差并转换为线上类型；`getSensorLink()` 从包内 isLeft 选择 B/D/F 或 C/E/G 结果变量并恢复实际单位。收发函数只负责帧头、字节缓存、校验、memcpy 和发送限频。修改协议时同步修改 SensorPayload、setSensorLink()、getSensorLink() 及手写的 printReceivedData()。

```cpp
receiveSensorData(); // A 依次轮询右、左对象，从机轮询一个对象。
forwardReceivedChildSensors(); // 从机向上级回传，A 和单机不发送。
isSensorDataFresh(true); // 查询一秒内收到的有效左链状态。
```

接收只有完整帧且校验及 bool 编码检查通过后才更新数据；半帧、坏帧不覆盖有效结果或刷新接收时间。两个 A 对象独立保存半帧；各自的 getSensorLink() 将数据发布到现有 phiB_raw、thetaB_raw、GYRO_X_B、B_ele_PWM 等接口，控制、日志及地面站的结果变量不变。getSensorLink() 根据包内左右标志发布数据；isSensorDataFresh() 直接按接收链路选择 left/right 对象，不再使用 isLeftSide()。

上行仍以 200 Hz 发送，本机之前的固定槽位置零，本机填实时 IMU 与物理升降舵 PWM，下游填收到的结果。相对角在转换为 int16_t 前用实际角度相减；MT6701 仅用于测量和记录。isDataFresh() 只判断对应对象的数据新鲜度，不清除旧结果或阻止继续汇总。

SD 参数 `usb_sensorlink_debug` 与命令打印采用相同的 0/1 开关、立即生效和延迟保存逻辑，默认 0。为 1 时按字段名手动打印收到的 SensorPayload 原始值。

## 命令打印

SD 参数 `usb_commandlink_debug` 属于 Debug 组，整数范围 0～1，默认 1。设为 1 时，每个有效上级命令帧打印 commandPayload 的字段名及原始协议数值；设为 0 时关闭打印。参数修改立即生效，通过现有 2 秒合并保存流程持久化到 SD。目标俯仰角 `pitchAngles` 的原始单位为 0.1 度，例如 123 表示 12.3 度。打印函数逐字段输出，不依赖动态分配；增删或修改结构体字段时同步更新打印函数。

## 统一命名

| 职责 | 命令链 | 状态链 |
| --- | --- | --- |
| 类型 | AircraftCommandLink | AircraftSensorLink |
| 实例入口 | CommandLinks::left/right | SensorLinks::left/right |
| 初始化 | initializeCommandLinks | initializeSensorLinks |
| 接收（对象及编排入口） | receiveCommandData | receiveSensorData |
| 输入到协议转换 | setCommandLink | setSensorLink |
| 协议到结果转换 | getCommandLink | getSensorLink |
| 转发 | forwardReceivedChildCommands | forwardReceivedChildSensors |
| 对象已收到有效帧 | hasReceivedData | hasReceivedData |
| 外部接收状态查询 | hasReceivedCommandData | isSensorDataFresh(left) |
| 打印 | printReceivedData | printReceivedData |
| SD 打印设置绑定 | commandLinkDebugSetting | sensorLinkDebugSetting |

两类 configure() 都按 output（发送串口）、input（接收串口）、链路配置的顺序传参；不存在的方向传 nullptr。共同成员统一为 transmitPort、receivePort、receiveBuffer、receivePosition、dataReceived、lastTransmitTimeUs、receivedFrameCount。状态链另有 lastReceiveTimeMs 和 isDataFresh() 用于时效判断，命令链只记录是否收到过有效帧，两者语义不变。协议中 FF 缩写统一为 Ff，字段 eleFfPwmCommands 的类型和位置不变。
