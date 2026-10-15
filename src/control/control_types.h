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
  float desiredRate = 0, filteredRate = 0;
  float desiredAcceleration = 0, usedAcceleration = 0;
  float commandedDeflection = 0, estimatedDeflection = 0;
};

struct IndiPitchControlOutput {
  float acceleration = 0; // deg/s^2
  float rate = 0;         // deg/s
  float measuredRate = 0;
  bool valid = false;
  int pwm = 0; // 当前周期按原标定和限幅得到的物理 PWM（us）。
  IndiPitchDiagnostics diagnostics;
};

struct ManualControlOutput {
  float roll = 0, pitch = 0, yaw = 0, throttle = 0;
};

struct PitchPidControlOutput {
  float command = 0;
};

struct RollDampingControlOutput {
  int command = 0; // 保留原阻尼输出尺度，单位 us。
};

struct RollControlOutput {
  float command = 0;
  float filteredCommand = 0, filteredDerivative = 0;
  float equivalentRoll = 0;
};

struct YawControlOutput {
  float command = 0;
};

struct ThrottleControlOutput {
  float command = 0;
};

struct ConfigurationControlOutput {
  float commands[6] = {}, errors[6] = {}, angles[6] = {}, rates[6] = {};
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
