#include "math/units.h"
#include "radio_comm/radio_comm.h"
#include "control/control_output.h"
#include "control/control_modes.h"
#include "parameters/control_parameters.h"
#include "diagnostics/diagnostic_frame.h"
#include "config/aircraft_config.h"
#include "diagnostics/debug_print.h"
#include "actuators/actuator_output.h"
#include "control/command_generation.h"
#include "system/flight_clock.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "sensors/sensor_processing.h"
#include <stdio.h>

namespace {
int32_t actuator_output_debug = 0;
int32_t control_mode_debug = 0;
int32_t airspeed_debug = 0;
int32_t radio_debug = 0;
int32_t desired_state_debug = 0;
int32_t configuration_debug = 0;
int32_t pid_output_debug = 0;
int32_t servo_commands_debug = 0;
int32_t loop_rate_debug = 0;
int32_t gyro_debug = 0;
int32_t accel_debug = 0;
int32_t attitude_debug = 0;
int32_t quaternion_debug = 0;
}

int32_t &actuatorOutputDebugSetting() { return actuator_output_debug; }
int32_t &controlModeDebugSetting() { return control_mode_debug; }
int32_t &airspeedDebugSetting() { return airspeed_debug; }
int32_t &radioDebugSetting() { return radio_debug; }
int32_t &desiredStateDebugSetting() { return desired_state_debug; }
int32_t &configurationDebugSetting() { return configuration_debug; }
int32_t &pidOutputDebugSetting() { return pid_output_debug; }
int32_t &servoCommandsDebugSetting() { return servo_commands_debug; }
int32_t &loopRateDebugSetting() { return loop_rate_debug; }
int32_t &gyroDebugSetting() { return gyro_debug; }
int32_t &accelDebugSetting() { return accel_debug; }
int32_t &attitudeDebugSetting() { return attitude_debug; }
int32_t &quaternionDebugSetting() { return quaternion_debug; }

void runDebugPrints() {
  printActuatorOutput(actuatorOutputDebugSetting());
  printControlMode(controlModeDebugSetting());
  printAirspeed(airspeedDebugSetting());
  printRadioData(radioDebugSetting());
  printDesiredState(desiredStateDebugSetting());
  printConfigurationData(configurationDebugSetting());
  printPIDoutput(pidOutputDebugSetting());
  printServoCommands(servoCommandsDebugSetting());
  printLoopRate(loopRateDebugSetting());
  printGyroData(gyroDebugSetting());
  printAccelData(accelDebugSetting());
  printRollPitchYaw(attitudeDebugSetting());
  printQuaternion(quaternionDebugSetting());
}

void printAirspeed(int enabled) {
  static uint32_t last = 0;
  if (enabled != 1) return;
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - last) < 100000UL) return;
  const AirspeedDiagnostics diagnostic = airspeedDiagnostics();
  char text[256];
  const int length = snprintf(text, sizeof(text),
      "[AIRSPEED] airspeed=%.3f (m/s) rx=%u read_ok=%u status=%u raw=%u "
      "pressure=%.2f zero=%.2f dp=%.2f (Pa) temp=%.2f (C) reads=%lu failed=%lu\n",
      airspeed, static_cast<unsigned>(diagnostic.receivedBytes),
      static_cast<unsigned>(diagnostic.readOk), static_cast<unsigned>(diagnostic.status),
      static_cast<unsigned>(diagnostic.pressureCounts), diagnostic.pressurePa,
      diagnostic.zeroPa, diagnostic.differentialPa, diagnostic.temperatureC,
      static_cast<unsigned long>(diagnostic.successfulReads),
      static_cast<unsigned long>(diagnostic.failedReads));
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(text)) return;
  if (!queueDiagnosticOutput(text, length)) return;
  last = now;
}

void printControlMode(int enabled) {
  static uint32_t last = 0;
  static uint32_t frame = 0;
  if (enabled != 1) return;
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - last) < 100000UL) return;
  char text[384];
  const int length = snprintf(text, sizeof(text),
      "\n=== ControlMode #%lu ===\n"
      "currentFlightMode=%d previousFlightMode=%d activePitchController=%d previousPitchController=%d\n"
      "mixerStrategyCommand=%.3f configurationControlScale=%.3f configurationAngle=%.6f (rad)\n"
      "brakeEnabled=%d controlModeChanged=%d integralControlEnabled=%d\n"
      "=============\n",
      static_cast<unsigned long>(frame + 1), static_cast<int>(currentFlightMode),
      static_cast<int>(previousFlightMode), static_cast<int>(activePitchController),
      static_cast<int>(previousPitchController), mixerStrategyCommand, ControlParameters::configurationControlScale,
      configurationAngle, static_cast<int>(brakeEnabled), static_cast<int>(controlModeChanged),
      static_cast<int>(ControlParameters::integralControlEnabled));
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(text)) return;
  if (!queueDiagnosticOutput(text, length)) return;
  last = now;
  ++frame;
}

void printActuatorOutput(int enabled) {
  static uint32_t last = 0;
  static uint32_t frame = 0;
  if (enabled != 1) return;
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - last) < 100000UL) return;
  char text[192];
  const int length = snprintf(text, sizeof(text),
      "\n=== ActuatorOutput #%lu ===\n"
      "ail1=%.0f ail2=%.0f ele=%.0f thro=%.0f rudd=%.0f (us)\n"
      "=============\n",
      static_cast<unsigned long>(frame + 1), ActuatorOutput[0], ActuatorOutput[1],
      ActuatorOutput[2], ActuatorOutput[3], ActuatorOutput[4]);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(text)) return;
  if (!queueDiagnosticOutput(text, length)) return;
  last = now;
  ++frame;
}

// 输出遥控各通道 PWM。
void printRadioData(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F(" CH1:"));
    output.print(radio_channel1_raw);
    output.print(F(" CH2:"));
    output.print(radio_channel2_raw);
    output.print(F(" CH3:"));
    output.print(radio_channel3_raw);
    output.print(F(" CH4:"));
    output.print(radio_channel4_raw);
    output.print(F(" CH5:"));
    output.print(radio_channel5_raw);
    output.print(F(" CH6:"));
    output.println(radio_channel6_raw);
  }
  output.send();
}

// 输出油门与三轴物理目标。
void printDesiredState(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F("targets.throttle:"));
    output.print(targets.throttle);
    output.print(F(" targets.roll:"));
    output.print(targets.roll);
    output.print(F(" targets.pitch:"));
    output.print(targets.pitch);
    output.print(F(" targets.yawRate:"));
    output.println(targets.yawRate);
  }
  output.send();
}

// 输出相邻机体夹角或原始姿态差，以及各子机滚转角速度。
void printConfigurationData(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F("phiab:"));
    output.print(att_B[0] - att[0]);
    output.print(F("phiac:"));
    output.print(att_C[0] - att[0]);
    output.print(F("phibd:"));
    output.print(att_D[0] - att_B[0]);
    output.print(F("phice:"));
    output.print(att_E[0] - att_C[0]);
    output.print(F("phidf:"));
    output.print(att_F[0] - att_D[0]);
    output.print(F("phieg:"));
    output.print(att_G[0] - att_E[0]);
    output.print(F("phieq:"));
    output.print(rollControlOutput.equivalentRollAngle);
    output.print(F("Pb:"));
    output.print(ang_vel_B[0]);
    output.print(F("Pc:"));
    output.print(ang_vel_C[0]);
    output.print(F("Pd:"));
    output.print(ang_vel_D[0]);
    output.print(F("Pe:"));
    output.print(ang_vel_E[0]);
    output.print(F("Pf:"));
    output.print(ang_vel_F[0]);
    output.print(F("Pg:"));
    output.println(ang_vel_G[0]);
  }
  output.send();
}

// 输出各轴与构型控制器的 PID 结果，供串口调试。
void printPIDoutput(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F("roll_PID:"));
    output.print(rollControlOutput.pidOutput);
    output.print(F(" pitch_PID:"));
    output.print(pitchPidControlOutput.pidOutput);
    output.print(F(" yawRate_PID:"));
    output.print(yawRateControlOutput.pidOutput);
    output.print(F(" Phiab_PID:"));
    output.print(configurationControlOutput.pidOutputs[0]);
    output.print(F(" Phiac_PID:"));
    output.println(configurationControlOutput.pidOutputs[1]);
    output.print(F(" Phibd_PID:"));
    output.print(configurationControlOutput.pidOutputs[2]);
    output.print(F(" Phice_PID:"));
    output.println(configurationControlOutput.pidOutputs[3]);
  }
  output.send();
}

// 输出各执行器 PWM 指令，供串口调试。
void printServoCommands(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F("A_ail1:"));
    output.print(ActuatorOutput[0]);
    output.print(F(" A_ail2:"));
    output.print(ActuatorOutput[1]);
    output.print(F(" A_ele:"));
    output.print(ActuatorOutput[2]);
    output.print(F(" A_thro:"));
    output.print(ActuatorOutput[3]);
    output.print(F(" A_rudd:"));
    output.print(ActuatorOutput[4]);
    // output.print(F(" s6_command:"));
    // output.print(s6_command_PWM);
    // output.print(F(" s7_command:"));
    // output.println(s7_command_PWM);
  }
  output.send();
}

// 输出主循环时间间隔 dt，单位为 μs；本函数不直接计算频率。
void printLoopRate(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F("dt:"));
    output.println(dt * 1000000.0); // 一个微秒microsecons=1/1000000s
  }
  output.send();
}


// 输出三轴角速度，单位为 rad/s。
void printGyroData(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F("ang_vel[0]:")); output.print(ang_vel[0]);
    output.print(F(" ang_vel[1]:")); output.print(ang_vel[1]);
    output.print(F(" ang_vel[2]:")); output.println(ang_vel[2]);
  }
  output.send();
}


// 输出三轴加速度，单位为 m/s²。
void printAccelData(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F("acc[0]:")); output.print(acc[0]);
    output.print(F(" acc[1]:")); output.print(acc[1]);
    output.print(F(" acc[2]:")); output.println(acc[2]);
  }
  output.send();
}

// 输出当前姿态解算的滚转、俯仰和偏航角，单位为 rad。
void printRollPitchYaw(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F("roll:"));
    output.print(att[0]);
    output.print(F(" pitch:"));
    output.print(att[1]);
    output.print(F(" yaw:"));
    output.println(att[2]);
  }
  output.send();
}


// 输出姿态估计使用的四元数分量。
void printQuaternion(int enabled) {
  if (enabled != 1) return;
  static uint32_t print_counter = 0;
  DiagnosticFrame output;
  if (static_cast<uint32_t>(micros() - print_counter) >= 100000UL) {
    print_counter = micros();
    output.print(F("quat[0]: "));
    output.print(quat[0]);
    output.print(F(" quat[1]: "));
    output.print(quat[1]);
    output.print(F(" quat[2]: "));
    output.print(quat[2]);
    output.print(F(" quat[3]: "));
    output.println(quat[3]);
  }
  output.send();
}
