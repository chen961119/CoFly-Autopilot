# 机间通讯

## 当前拆分

| 文件 | 职责 |
| --- | --- |
| `interaircraft_comm.cpp` | 上级命令和左右状态的接收、校验、解析；命令准备、下发、转发与状态回传；收发缓存及限频 |
| `interaircraft_common.cpp` | 串口初始化、本机翼尖相对角查询、现有公共兼容变量 |
| `interaircraft_comm.h` | 控制、执行器、日志与地面站使用的公共接口 |

接收与发送合并在同一实现文件，直接使用同一份私有 `recvData`，不再通过 `receivedParentCommand()` 获取缓存，也不再保留内部共享头文件。解析器字节缓存、收发时间和限频状态均留在该文件中。目前尚未将现有 `Local_*`、`phiB_raw` 等公共变量改成下面的结构体。

## 结构体整理方案（待实现）

建议按“单机业务数据 → 三机子链 → 收发缓存”组织，避免把所有变量简单包进一个大结构体。

```cpp
struct AircraftCommand {
  int16_t aileron1OffsetUs;
  int16_t aileron2OffsetUs;
  int16_t throttleOffsetUs;
  int16_t rudderOffsetUs;
  int16_t elevatorManualOffsetUs;
  int16_t elevatorFeedForwardOffsetUs;
  float pitchDeg;
  bool light;
};

struct AircraftState {
  float rollDeg;
  float pitchDeg;
  float rollRateDegPerSec;
  int16_t elevatorPwmUs;
};

struct BranchCommand {
  AircraftCommand aircraft[3];
  bool integralEnabled;
  bool forceManual;
};

struct BranchState {
  AircraftState aircraft[3];
  float jointAngleDeg[3];
};

struct LinkStatus {
  bool received;
  uint32_t lastValidRxMs;
};

struct ReceiveData {
  BranchCommand parentCommand;
  BranchState leftState;
  BranchState rightState;
  LinkStatus parent;
  LinkStatus left;
  LinkStatus right;
};

struct TransmitData {
  BranchCommand leftCommand;
  BranchCommand rightCommand;
  BranchState upstreamState;
};
```

这些是 RAM 业务结构，不是串口打包结构；发送仍逐字段编码，不直接发送 `sizeof(struct)`。因此结构体对齐和 `bool` 大小不会改变现有协议。

### 变量映射

| 当前变量 | 建议位置 |
| --- | --- |
| `recvData.servo[12]`、`pitch[3]`、`ele_pwm[3]`、`ele_ff_pwm[3]`、`lights[3]` | `rx.parentCommand.aircraft[0..2]` 对应字段 |
| `recvData.int_is_valid / Force_manual` | `rx.parentCommand.integralEnabled / forceManual` |
| `phiB_raw / thetaB_raw / GYRO_X_B / B_ele_PWM` | `rx.leftState.aircraft[0]`；D/F 对应 1/2 |
| `phiC_raw / thetaC_raw / GYRO_X_C / C_ele_PWM` | `rx.rightState.aircraft[0]`；E/G 对应 1/2 |
| `servoCommandsleft/right` 等并行数组 | `tx.leftCommand / tx.rightCommand` |
| `RelativeAngleAll / GYROAll / PHIALL / THETAALL / ELEPWM` | `tx.upstreamState` |
| 接收标志与时间 | `rx.parent / left / right` |
| `Local_*_control_us / Local_pitch_des` | 统一的本机控制命令；A 由控制器生成，从机由上级命令中选出。它是控制输入/输出接口，不混入子链状态 |

### 归属与读取规则

1. 左链固定索引为 B、D、F，右链为 C、E、G；当前固定长度帧的空位置继续保留，不在转发时压缩或挪动位置。用 `AircraftId` 和查询函数封装索引，避免业务代码写 `servo[i * 4 + 2]`。
2. 接收器先在局部候选结构解析完整帧，通过校验后一次提交到 `ReceiveData` 并更新时间。坏帧、半帧不能覆盖上一份有效结果。对外提供 `const` 引用/指针，业务代码不能写接收缓存。
3. 发送缓存由控制器的准备接口写入，由发送器读取编码。上级转发从接收缓存读取，并写入自己的发送缓存；不把 RX 缓存当 TX 临时变量使用。
4. 控制偏移使用有符号微秒，实际升降舵输出使用 PWM 微秒，角度使用度，滚转角速度使用度/秒。字段名保留单位，避免旧 `PWM` 名称混淆偏移与实际输出。
5. 相对角和相对角速度由对应两机 IMU 姿态/角速度计算；MT6701 仍只测量和记录。状态帧里已有的相对角可留作收到的数据，但控制使用统一的姿态差计算入口。
6. 链路新鲜度属于收到该帧的串口链路。目前协议不能证明每个远端节点都在同一时刻更新，不给 D/F/E/G 凭空增加“独立有效时间”。若以后要逐节点判断，需扩展状态协议携带有效标记或时间。
7. 串口字节缓存、解析位置、限频时间等是私有运行状态，留在收发实现文件，不放入公开的结果结构。收发使用同一文件不影响数据归属与结构体分组。
8. 初始值与有效标志必须明确：未收帧时不能应用空命令，默认本机油门继续为 `1000 - PWM_CENTER_US`；链路失效沿用现有保护规则。控制运算保持原来的计算宽度，编码为 `int16_t` 前检查范围。

### 实施次序

先添加业务结构与只读查询接口，再迁移收发编码和转发，最后更新控制、执行器、日志、地面站及诊断的调用，并删除旧公共变量。最终只保留一份真实数据源，不长期维护“结构体 + 旧变量”两套值。本次尚未实施这一步。
