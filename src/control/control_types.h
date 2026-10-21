#pragma once

#include "config/flight_config.h"

enum class PitchController { PID = 0, INDI = 1 };

// 所有模式：roll/pitch 为角度（rad），yawRate 为偏航角速度（rad/s），throttle 为 0～1。
// 从机接收的 pitch 同样为 rad；主机遥控直通量另存于 manual_input。
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
  float indiOutput = 0; // 模式0：抬头正、0=配平的归一化俯仰控制量，mixer取负后使用；不是角加速度。模式1为-1..0。
  float q_dot_meas = 0; // q_dot：传感器层给出的俯仰角加速度反馈，rad/s^2，已差分滤波。
  float delta_e_cmd = 0; // 限速、一次低通及通道3范围限幅后的命令舵偏，rad，下偏为正。
  float delta_e_hat = 0; // 延迟及一阶舵机模型更新后的舵偏估计，rad，下偏为正；不是实测舵角。
};

struct PitchPidControlOutput {
  float pidOutput = 0; // 俯仰 PID 控制意图：正值要求抬头，由分配器转换为下偏正的舵面量。
};

struct RollDampingControlOutput {
  float dampingCommand = 0; // 归一化滚转阻尼指令：与舵面命令相加，最终按本机两侧行程映射。
};

struct RollControlOutput {
  float pidOutput = 0; // 滚转 PID 控制量，供后续混控分配。
  float filteredPidOutput = 0; // PID 控制量的低通滤波值。
  float filteredPidOutputDerivative = 0; // PID 控制量对时间的导数，经低通滤波。
  float equivalentRollAngle = 0; // 反馈使用的等效滚转角，单位 rad；单机时为本机滚转角。
};

struct YawRateControlOutput {
  float pidOutput = 0; // 偏航角速度 PID 控制量，供后续混控分配（当前实际执行器为油门差动）。
  // 已乘对应增益的控制分量；四项之和等于 pidOutput，未启用时均为 0。
  float proportionalTerm = 0;
  float integralTerm = 0; // 乘 Ki 后限幅的 I 项。
  float derivativeTerm = 0;
  float feedforwardTerm = 0;
};

struct ThrottleControlOutput {
  float throttleCommand = 0; // 基础油门 0～1，随后仍由 mixer 叠加差动。
  float airspeedTarget = 0; // m/s；仅空速闭环启用时有效。
  float filteredAirspeed = 0; // m/s；无有效反馈时为 0。
  float airspeedError = 0; // m/s，目标减反馈。
  float integralTerm = 0; // 油门尺度的 I 项。
  bool speedControlActive = false;
};

struct ConfigurationControlOutput {
  // 各数组顺序均为 AB、AC、BD、CE、DF、EG。
  float pidOutputs[6] = {}; // 各构型通道的 PID 控制量。
  float angleErrors[6] = {}; // rad； 目标构型角减去测量构型角。
  float measuredAngles[6] = {}; // rad； 相连两机的滚转角之差。
  float measuredRates[6] = {}; // rad/s； 相连两机的滚转角速度之差。
};

// 舵面逻辑量为 -1..1，油门为 0..1；不含本机 PWM 参数，混控贡献可超过标准范围。
// aileron1、aileron2 和 elevator 均以舵面下偏为正；elevator 不是俯仰角。
// 每路 mode=0 接收 -1..1 并使用 min/trim/max，mode=1 接收 0..1 并使用 min/max。
// 当前舵面控制输出匹配 mode=0，油门输出匹配 mode=1；调用方负责匹配输入范围。
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
