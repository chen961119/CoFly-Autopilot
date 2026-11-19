#include "math/units.h"
#include "logging/flight_service_sample.h"
#include "logging/logger.h"
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
#include "system/flight_tasks.h"
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
  FlightServiceSample sample;
  copyFlightServiceSample(sample);
  const AirspeedDiagnostics diagnostic = sample.airspeedStatus;
  char text[256];
  const int length = snprintf(text, sizeof(text),
      "[AIRSPEED] airspeed=%.3f (m/s) rx=%u read_ok=%u status=%u raw=%u "
      "pressure=%.2f zero=%.2f dp=%.2f (Pa) temp=%.2f (C) reads=%lu failed=%lu\n",
      sample.airspeed, static_cast<unsigned>(diagnostic.receivedBytes),
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
  FlightServiceSample sample;
  copyFlightServiceSample(sample);
  char text[384];
  const int length = snprintf(text, sizeof(text),
      "\n=== ControlMode #%lu ===\n"
      "currentFlightMode=%d previousFlightMode=%d activePitchController=%d previousPitchController=%d\n"
      "mixerStrategyCommand=%.3f configurationControlScale=%.3f configurationAngle=%.6f (rad)\n"
      "brakeEnabled=%d controlModeChanged=%d integralControlEnabled=%d\n"
      "=============\n",
      static_cast<unsigned long>(frame + 1), static_cast<int>(sample.currentFlightMode),
      static_cast<int>(sample.previousFlightMode), static_cast<int>(sample.activePitchController),
      static_cast<int>(sample.previousPitchController), sample.mixerStrategyCommand, sample.configurationControlScale,
      sample.configurationAngle, static_cast<int>(sample.brakeEnabled), static_cast<int>(sample.controlModeChanged),
      static_cast<int>(sample.integralControlEnabled));
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
  FlightServiceSample sample;
  copyFlightServiceSample(sample);
  char text[192];
  const int length = snprintf(text, sizeof(text),
      "\n=== ActuatorOutput #%lu ===\n"
      "ail1=%.0f ail2=%.0f ele=%.0f thro=%.0f rudd=%.0f (us)\n"
      "=============\n",
      static_cast<unsigned long>(frame + 1), sample.ActuatorOutput[0], sample.ActuatorOutput[1],
      sample.ActuatorOutput[2], sample.ActuatorOutput[3], sample.ActuatorOutput[4]);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F(" CH1:"));
    output.print(sample.radioRaw[0]);
    output.print(F(" CH2:"));
    output.print(sample.radioRaw[1]);
    output.print(F(" CH3:"));
    output.print(sample.radioRaw[2]);
    output.print(F(" CH4:"));
    output.print(sample.radioRaw[3]);
    output.print(F(" CH5:"));
    output.print(sample.radioRaw[4]);
    output.print(F(" CH6:"));
    output.println(sample.radioRaw[5]);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F("targets.throttle:"));
    output.print(sample.targets.throttle);
    output.print(F(" targets.roll:"));
    output.print(sample.targets.roll);
    output.print(F(" targets.pitch:"));
    output.print(sample.targets.pitch);
    output.print(F(" targets.yawRate:"));
    output.println(sample.targets.yawRate);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F("phiab:"));
    output.print(sample.wingAttitude[0][0] - sample.att[0]);
    output.print(F("phiac:"));
    output.print(sample.wingAttitude[1][0] - sample.att[0]);
    output.print(F("phibd:"));
    output.print(sample.wingAttitude[2][0] - sample.wingAttitude[0][0]);
    output.print(F("phice:"));
    output.print(sample.wingAttitude[3][0] - sample.wingAttitude[1][0]);
    output.print(F("phidf:"));
    output.print(sample.wingAttitude[4][0] - sample.wingAttitude[2][0]);
    output.print(F("phieg:"));
    output.print(sample.wingAttitude[5][0] - sample.wingAttitude[3][0]);
    output.print(F("phieq:"));
    output.print(sample.rollControlOutput.equivalentRollAngle);
    output.print(F("Pb:"));
    output.print(sample.wingRollRate[0]);
    output.print(F("Pc:"));
    output.print(sample.wingRollRate[1]);
    output.print(F("Pd:"));
    output.print(sample.wingRollRate[2]);
    output.print(F("Pe:"));
    output.print(sample.wingRollRate[3]);
    output.print(F("Pf:"));
    output.print(sample.wingRollRate[4]);
    output.print(F("Pg:"));
    output.println(sample.wingRollRate[5]);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F("roll_PID:"));
    output.print(sample.rollControlOutput.pidOutput);
    output.print(F(" pitch_PID:"));
    output.print(sample.pitchPidControlOutput.pidOutput);
    output.print(F(" yawRate_PID:"));
    output.print(sample.yawRateControlOutput.pidOutput);
    output.print(F(" Phiab_PID:"));
    output.print(sample.configurationControlOutput.pidOutputs[0]);
    output.print(F(" Phiac_PID:"));
    output.println(sample.configurationControlOutput.pidOutputs[1]);
    output.print(F(" Phibd_PID:"));
    output.print(sample.configurationControlOutput.pidOutputs[2]);
    output.print(F(" Phice_PID:"));
    output.println(sample.configurationControlOutput.pidOutputs[3]);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F("A_ail1:"));
    output.print(sample.ActuatorOutput[0]);
    output.print(F(" A_ail2:"));
    output.print(sample.ActuatorOutput[1]);
    output.print(F(" A_ele:"));
    output.print(sample.ActuatorOutput[2]);
    output.print(F(" A_thro:"));
    output.print(sample.ActuatorOutput[3]);
    output.print(F(" A_rudd:"));
    output.print(sample.ActuatorOutput[4]);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F("dt:"));
    output.print(sample.dt * 1000000.0);
    const FlightTimingStats stats = flightTimingStats();
    output.print(F(" mean_us:")); output.print(stats.cycles ? static_cast<float>(stats.totalPeriodUs) / stats.cycles : 0.0f);
    output.print(F(" max_period_us:")); output.print(stats.maxPeriodUs);
    output.print(F(" max_execution_us:")); output.print(stats.maxExecutionUs);
    output.print(F(" imu_pwm_us:")); output.print(stats.lastImuToPwmUs);
    output.print(F(" max_imu_pwm_us:")); output.print(stats.maxImuToPwmUs);
    output.print(F(" cycles:")); output.print(stats.cycles);
    output.print(F(" missed:")); output.print(stats.missedReleases);
    output.print(F(" intervals_ge_4ms:")); output.print(stats.intervalsAtLeast4ms);
    output.print(F(" execution_gt_2ms:")); output.print(stats.executionsOver2ms);
    output.print(F(" log_drops:")); output.print(droppedFlightLogSamples());
    output.print(F(" log_write_errors:")); output.print(flightLogWriteFailures());
    const FlightStackMargins stacks = flightStackMargins();
    output.print(F(" stack_words(control/links/aux/params/storage):"));
    output.print(stacks.control); output.print('/');
    output.print(stacks.links); output.print('/');
    output.print(stacks.auxiliary); output.print('/');
    output.print(stacks.parameters); output.print('/');
    output.println(stacks.storage);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F("ang_vel[0]:")); output.print(sample.ang_vel[0]);
    output.print(F(" ang_vel[1]:")); output.print(sample.ang_vel[1]);
    output.print(F(" ang_vel[2]:")); output.println(sample.ang_vel[2]);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F("acc[0]:")); output.print(sample.acc[0]);
    output.print(F(" acc[1]:")); output.print(sample.acc[1]);
    output.print(F(" acc[2]:")); output.println(sample.acc[2]);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F("roll:"));
    output.print(sample.att[0]);
    output.print(F(" pitch:"));
    output.print(sample.att[1]);
    output.print(F(" yaw:"));
    output.println(sample.att[2]);
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
    FlightServiceSample sample;
    copyFlightServiceSample(sample);
    output.print(F("quat[0]: "));
    output.print(sample.quat[0]);
    output.print(F(" quat[1]: "));
    output.print(sample.quat[1]);
    output.print(F(" quat[2]: "));
    output.print(sample.quat[2]);
    output.print(F(" quat[3]: "));
    output.println(sample.quat[3]);
  }
  output.send();
}
