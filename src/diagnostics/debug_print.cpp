#include "radio_comm/radio_comm.h"
#include "control/control_output.h"
#include "control/control_modes.h"
#include "parameters/control_parameters.h"
#include "config/serial_config.h"
#include "config/aircraft_config.h"
#include "diagnostics/debug_print.h"
#include "actuators/actuator_output.h"
#include "control/command_generation.h"
#include "system/flight_clock.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "sensors/sensor_processing.h"
#include <stdio.h>

// 旧诊断函数共用节流时间戳；执行器和油门打印独立计时。
static unsigned long print_counter = 0;

namespace {
int32_t actuator_output_debug = 0;
int32_t control_mode_debug = 0;
}

int32_t &actuatorOutputDebugSetting() { return actuator_output_debug; }
int32_t &controlModeDebugSetting() { return control_mode_debug; }

void printControlMode(int enabled) {
  static uint32_t last = 0;
  static uint32_t frame = 0;
  if (enabled != 1 || !USBSerial) return;
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - last) < 100000UL) return;
  char text[384];
  const int length = snprintf(text, sizeof(text),
      "\n=== ControlMode #%lu ===\n"
      "currentFlightMode=%d previousFlightMode=%d activePitchController=%d previousPitchController=%d\n"
      "mixerStrategyCommand=%.3f configurationControlScale=%.3f configurationAngleDeg=%.1f\n"
      "brakeEnabled=%d controlModeChanged=%d integralControlEnabled=%d\n"
      "=============\n",
      static_cast<unsigned long>(frame + 1), static_cast<int>(currentFlightMode),
      static_cast<int>(previousFlightMode), static_cast<int>(activePitchController),
      static_cast<int>(previousPitchController), mixerStrategyCommand, configurationControlScale,
      configurationAngleDeg, static_cast<int>(brakeEnabled), static_cast<int>(controlModeChanged),
      static_cast<int>(ControlParameters::integralControlEnabled));
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(text) ||
      USBSerial.availableForWrite() < length) return;
  USBSerial.write(reinterpret_cast<const uint8_t *>(text), length);
  last = now;
  ++frame;
}

void printActuatorOutput(int enabled) {
  static uint32_t last = 0;
  static uint32_t frame = 0;
  if (enabled != 1 || !USBSerial) return;
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - last) < 100000UL) return;
  char text[192];
  const int length = snprintf(text, sizeof(text),
      "\n=== ActuatorOutput #%lu ===\n"
      "ail1=%.0f ail2=%.0f ele=%.0f thro=%.0f rudd=%.0f (us)\n"
      "=============\n",
      static_cast<unsigned long>(frame + 1), ActuatorOutput[0], ActuatorOutput[1],
      ActuatorOutput[2], ActuatorOutput[3], ActuatorOutput[4]);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(text) ||
      USBSerial.availableForWrite() < length) return;
  USBSerial.write(reinterpret_cast<const uint8_t *>(text), length);
  last = now;
  ++frame;
}

// 输出上级接收的油门逻辑控制量。
static int32_t throttle_debug = 0;
int32_t &localThrottleDebugSetting() { return throttle_debug; }
void printLocalThrottle() {
  static unsigned long last = 0;
  if (throttle_debug == 0 || micros() - last < 100000UL) return;
  last = micros();
  const int room = USBSerial.availableForWrite();
  if (room < 80) return;
  AircraftCommandLink *link = aircraftIsLeft() ? CommandLinks::left : CommandLinks::right;
  const float throttle = aircraftId() != 1 && link
      ? link->getCommandLink().masterContribution.throttle : -0.5f;
  USBSerial.printf("[THROTTLE] parent_received=%d control=%.6f\n",
                hasReceivedCommandData() ? 1 : 0, throttle);
}

// 按共享时间戳节流输出遥控各通道 PWM。
void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F(" CH1:"));
    USBSerial.print(radio_channel1_raw);
    USBSerial.print(F(" CH2:"));
    USBSerial.print(radio_channel2_raw);
    USBSerial.print(F(" CH3:"));
    USBSerial.print(radio_channel3_raw);
    USBSerial.print(F(" CH4:"));
    USBSerial.print(radio_channel4_raw);
    USBSerial.print(F(" CH5:"));
    USBSerial.print(radio_channel5_raw);
    USBSerial.print(F(" CH6:"));
    USBSerial.println(radio_channel6_raw);
  }
}

// 按共享时间戳节流输出油门与三轴期望状态。
void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("targets.throttle:"));
    USBSerial.print(targets.throttle);
    USBSerial.print(F(" targets.roll:"));
    USBSerial.print(targets.roll);
    USBSerial.print(F(" targets.pitch:"));
    USBSerial.print(targets.pitch);
    USBSerial.print(F(" targets.yaw:"));
    USBSerial.println(targets.yaw);
  }
}

// 输出相邻机体夹角或原始姿态差，以及各子机滚转角速度。
void printConfigurationData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("phiab:"));
    USBSerial.print(att_B[0] - att[0]);
    USBSerial.print(F("phiac:"));
    USBSerial.print(att_C[0] - att[0]);
    USBSerial.print(F("phibd:"));
    USBSerial.print(att_D[0] - att_B[0]);
    USBSerial.print(F("phice:"));
    USBSerial.print(att_E[0] - att_C[0]);
    USBSerial.print(F("phidf:"));
    USBSerial.print(att_F[0] - att_D[0]);
    USBSerial.print(F("phieg:"));
    USBSerial.print(att_G[0] - att_E[0]);
    USBSerial.print(F("phieq:"));
    USBSerial.print(rollControlOutput.equivalentRollAngleDeg);
    USBSerial.print(F("Pb:"));
    USBSerial.print(ang_vel_B[0]);
    USBSerial.print(F("Pc:"));
    USBSerial.print(ang_vel_C[0]);
    USBSerial.print(F("Pd:"));
    USBSerial.print(ang_vel_D[0]);
    USBSerial.print(F("Pe:"));
    USBSerial.print(ang_vel_E[0]);
    USBSerial.print(F("Pf:"));
    USBSerial.print(ang_vel_F[0]);
    USBSerial.print(F("Pg:"));
    USBSerial.println(ang_vel_G[0]);
  }
}

// 输出各轴与构型控制器的 PID 结果，供串口调试。
void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("roll_PID:"));
    USBSerial.print(rollControlOutput.pidOutput);
    USBSerial.print(F(" pitch_PID:"));
    USBSerial.print(pitchPidControlOutput.pidOutput);
    USBSerial.print(F(" yaw_PID:"));
    USBSerial.print(yawControlOutput.pidOutput);
    USBSerial.print(F(" Phiab_PID:"));
    USBSerial.print(configurationControlOutput.pidOutputs[0]);
    USBSerial.print(F(" Phiac_PID:"));
    USBSerial.println(configurationControlOutput.pidOutputs[1]);
    USBSerial.print(F(" Phibd_PID:"));
    USBSerial.print(configurationControlOutput.pidOutputs[2]);
    USBSerial.print(F(" Phice_PID:"));
    USBSerial.println(configurationControlOutput.pidOutputs[3]);
  }
}

// 输出各执行器 PWM 指令，供串口调试。
void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("A_ail1:"));
    USBSerial.print(ActuatorOutput[0]);
    USBSerial.print(F(" A_ail2:"));
    USBSerial.print(ActuatorOutput[1]);
    USBSerial.print(F(" A_ele:"));
    USBSerial.print(ActuatorOutput[2]);
    USBSerial.print(F(" A_thro:"));
    USBSerial.print(ActuatorOutput[3]);
    USBSerial.print(F(" A_rudd:"));
    USBSerial.print(ActuatorOutput[4]);
    // USBSerial.print(F(" s6_command:"));
    // USBSerial.print(s6_command_PWM);
    // USBSerial.print(F(" s7_command:"));
    // USBSerial.println(s7_command_PWM);
  }
}

// 输出主循环时间间隔 dt，单位为 μs；本函数不直接计算频率。
void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("dt:"));
    USBSerial.println(dt * 1000000.0); // 一个微秒microsecons=1/1000000s
  }
}


// 输出内置 IMU 三轴角速度，单位为 °/s。
void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("ang_vel[0]:")); USBSerial.print(ang_vel[0]);
    USBSerial.print(F(" ang_vel[1]:")); USBSerial.print(ang_vel[1]);
    USBSerial.print(F(" ang_vel[2]:")); USBSerial.println(ang_vel[2]);
  }
}


// 输出内置 IMU 三轴加速度，单位为 g。
void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("acc[0]:")); USBSerial.print(acc[0]);
    USBSerial.print(F(" acc[1]:")); USBSerial.print(acc[1]);
    USBSerial.print(F(" acc[2]:")); USBSerial.println(acc[2]);
  }
}





// 输出当前姿态解算的滚转、俯仰和偏航角，单位为度。
void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("roll:"));
    USBSerial.print(att[0]);
    USBSerial.print(F(" pitch:"));
    USBSerial.print(att[1]);
    USBSerial.print(F(" yaw:"));
    USBSerial.println(att[2]);
  }
}


// 输出姿态估计使用的四元数分量。
void printQuaternion() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    USBSerial.print(F("quat[0]: "));
    USBSerial.print(quat[0]);
    USBSerial.print(F(" quat[1]: "));
    USBSerial.print(quat[1]);
    USBSerial.print(F(" quat[2]: "));
    USBSerial.print(quat[2]);
    USBSerial.print(F(" quat[3]: "));
    USBSerial.println(quat[3]);
  }
}
