#pragma once

#include "config/flight_config.h"

enum class PitchController { PID = 0, INDI = 1 };

// 主机手动模式：roll/pitch/yawRate 为 ±1，控制器除以 2。
// 增稳模式：roll/pitch 为角度（rad），yawRate 为偏航角速度（rad/s），不是偏航角。
// 从机接收的 pitch 始终为俯仰角（rad）；throttle 始终为 0～1。
struct ControlTargets {
  float roll = 0, pitch = 0, yawRate = 0, throttle = 0;
  float configuration[6] = {}; // 构型角目标（rad），顺序 AB、AC、BD、CE、DF、EG。
};

using MasterControlTargets = ControlTargets;

struct SlaveControlTargets {
  float pitch = 0; // 从机俯仰角目标，rad。
};

// 从机固定索引 0～5 对应 B～G；A 主机单独存储，不占从机槽位。
constexpr unsigned CONTROL_AIRCRAFT_SLOTS = 6;
struct IndiPitchControlOutput {
  // INDI 符号：q=机体俯仰角速度，q_dot_ref=虚拟控制量，delta_e=升降舵偏角。
  // ref=参考值，f=滤波值，meas=测量值，hat=模型估计值，cmd=命令值。
  float q_dot_ref = 0; // q_dot_ref：目标俯仰角加速度，rad/s^2，不是舵偏或速度。
  float q_ref = 0; // q_ref：机体俯仰角速度参考值，rad/s，不是欧拉俯仰角变化率。
  float q_f = 0; // q_f：控制器专用低通后的机体俯仰角速度反馈，rad/s。
  bool isValid = false; // 本周期是否完成有效 INDI 控制计算。
  float indiOutput = 0; // 既有混控尺度：0=配平零舵角，1=1000 us逻辑偏移，下偏为正；上下偏统一系数1000*k。
  float q_dot_meas = 0; // q_dot：传感器层给出的俯仰角加速度反馈，rad/s^2，已差分滤波。
  float delta_e_cmd = 0; // 限速、一次低通及通道3范围限幅后的命令舵偏，rad，下偏为正。
  float delta_e_hat = 0; // 延迟及一阶舵机模型更新后的舵偏估计，rad，下偏为正；不是实测舵角。
};

struct ManualControlOutput {
  float rollCommand = 0, pitchCommand = 0, yawCommand = 0; // 手动控制意图，遥控输入除以 2；pitch 正值要求抬头。
  float throttleCommand = 0; // 手动油门指令，范围 0～1。
};

struct PitchPidControlOutput {
  float pidOutput = 0; // 俯仰 PID 控制意图：正值要求抬头，由分配器转换为下偏正的舵面量。
};

struct RollDampingControlOutput {
  float dampingCommand = 0; // 归一化滚转阻尼指令：1 对应 1000 us 的逻辑控制偏移。
};

struct RollControlOutput {
  float pidOutput = 0; // 滚转 PID 控制量，供后续混控分配。
  float filteredPidOutput = 0; // PID 控制量的低通滤波值。
  float filteredPidOutputDerivative = 0; // PID 控制量对时间的导数，经低通滤波。
  float equivalentRollAngle = 0; // 反馈使用的等效滚转角，单位 rad；单机时为本机滚转角。
};

struct YawRateControlOutput {
  float pidOutput = 0; // 偏航角速度 PID 控制量，供后续混控分配（当前实际执行器为油门差动）。
};

struct ThrottleControlOutput {
  float throttleCommand = 0; // 油门直通指令，沿用目标的 0～1 尺度。
};

struct ConfigurationControlOutput {
  // 各数组顺序均为 AB、AC、BD、CE、DF、EG。
  float pidOutputs[6] = {}; // 各构型通道的 PID 控制量。
  float angleErrors[6] = {}; // rad； 目标构型角减去测量构型角。
  float measuredAngles[6] = {}; // rad； 相连两机的滚转角之差。
  float measuredRates[6] = {}; // rad/s； 相连两机的滚转角速度之差。
};

// 统一的逻辑控制偏移：1 对应 1000 us，不含中位、安装 rev 或 trim。
// aileron1、aileron2 和 elevator 均以舵面下偏为正；elevator 不是俯仰角。
// 保留旧油门尺度；它不是新定义的推力物理量。
struct ActuatorCommand {
  float aileron1 = 0, aileron2 = 0, elevator = 0, throttle = 0, rudder = 0;
};

// 主机生成给从机的指令，后续通过机间通讯发给从机
struct SlaveCommand {
  FlightMode mode = MANUAL_MODE;
  SlaveControlTargets target;
  ActuatorCommand masterContribution; // 从作动器指令-1：手动为全量，增稳升降舵为前馈；elevator 始终下偏为正。
};

struct ControlCycleOutput {
  ActuatorCommand local;
  ActuatorCommand slaves[CONTROL_AIRCRAFT_SLOTS];
};
