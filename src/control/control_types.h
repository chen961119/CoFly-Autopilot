#pragma once

#include "config/flight_config.h"

enum class PitchController { PID = 0, INDI = 1 };

// 主机手动模式：roll/pitch/yaw 为 ±1，控制器除以 2；增稳模式为度制。
// 从机接收的 pitch 始终为俯仰角（度）；throttle 始终为 0～1。
struct ControlTargets {
  float roll = 0, pitch = 0, yaw = 0, throttle = 0;
  float configuration[6] = {}; // 构型角目标（度），顺序 AB、AC、BD、CE、DF、EG。
};

using MasterControlTargets = ControlTargets;

struct SlaveControlTargets {
  float pitch = 0; // 从机俯仰角目标，度。
};

// 从机固定索引 0～5 对应 B～G；A 主机单独存储，不占从机槽位。
constexpr unsigned CONTROL_AIRCRAFT_SLOTS = 6;
struct IndiPitchDiagnostics {
  float desiredRateDegPerSec = 0, filteredRateDegPerSec = 0; // 机体俯仰角速度目标与滤波测量值。
  float desiredAngularAccelerationDegPerSec2 = 0, measuredAngularAccelerationDegPerSec2 = 0;
  float commandedElevatorDeflectionDeg = 0, estimatedElevatorDeflectionDeg = 0;
};

struct IndiPitchControlOutput {
  float desiredAngularAccelerationDegPerSec2 = 0; // 机体俯仰角加速度目标。
  float desiredRateDegPerSec = 0; // 机体俯仰角速度目标。
  float filteredRateDegPerSec = 0; // 机体俯仰角速度的滤波测量值。
  bool isValid = false; // 本周期是否完成有效 INDI 控制计算。
  int elevatorPwmUs = 0; // 当前周期按原标定和限幅得到的升降舵物理 PWM。
  IndiPitchDiagnostics diagnostics;
};

struct ManualControlOutput {
  float rollCommand = 0, pitchCommand = 0, yawCommand = 0; // 手动逻辑控制量，遥控输入除以 2。
  float throttleCommand = 0; // 手动油门指令，范围 0～1。
};

struct PitchPidControlOutput {
  float pidOutput = 0; // 俯仰 PID 控制量，供后续混控分配。
};

struct RollDampingControlOutput {
  float dampingCommand = 0; // 归一化滚转阻尼指令：1 对应 1000 us 的逻辑控制偏移。
};

struct RollControlOutput {
  float pidOutput = 0; // 滚转 PID 控制量，供后续混控分配。
  float filteredPidOutput = 0; // PID 控制量的低通滤波值。
  float filteredPidOutputDerivative = 0; // PID 控制量对时间的导数，经低通滤波。
  float equivalentRollAngleDeg = 0; // 反馈使用的等效滚转角，单位 deg；单机时为本机滚转角。
};

struct YawControlOutput {
  float pidOutput = 0; // 偏航 PID 控制量，供后续混控分配。
};

struct ThrottleControlOutput {
  float throttleCommand = 0; // 油门直通指令，沿用目标的 0～1 尺度。
};

struct ConfigurationControlOutput {
  // 各数组顺序均为 AB、AC、BD、CE、DF、EG。
  float pidOutputs[6] = {}; // 各构型通道的 PID 控制量。
  float angleErrorsDeg[6] = {}; // 目标构型角减去测量构型角。
  float measuredAnglesDeg[6] = {}; // 相连两机的滚转角之差。
  float measuredRatesDegPerSec[6] = {}; // 相连两机的滚转角速度之差。
};

// 统一的逻辑控制偏移：1 对应 1000 us，不含中位、安装 rev 或 trim。
// 保留旧油门尺度；它不是新定义的推力物理量。
struct ActuatorCommand {
  float aileron1 = 0, aileron2 = 0, elevator = 0, throttle = 0, rudder = 0;
};

// 主机生成给从机的指令，后续通过机间通讯发给从机
struct SlaveCommand {
  FlightMode mode = MANUAL_MODE;
  SlaveControlTargets target;
  ActuatorCommand masterContribution; // 从作动器指令-1：手动为全量，增稳升降舵为前馈。
};

struct ControlCycleOutput {
  ActuatorCommand local;
  ActuatorCommand slaves[CONTROL_AIRCRAFT_SLOTS];
};
