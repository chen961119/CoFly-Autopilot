#include "radio_comm/radio_comm.h"
#include "control/command_generation.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "config/aircraft_config.h"
#include "control/control_modes.h"
#include "parameters/control_parameters.h"
#include "interface/human_interface.h"

FlightMode currentMode = MANUAL_MODE;
FlightMode lastMode = MANUAL_MODE;
PitchController pitchController = PitchController::PID;
PitchController lastPitchController = PitchController::PID;
SlaveCommand slaveCommand{MANUAL_MODE, {}, {0, 0, 0, -0.5f, 0}};

float mixerStrategy = 0.0f;
float configurationCoeff = 1.0f;
float configurationAngle = 0.0f;
bool brake = false;
bool changed = false;
bool integralEnabled = false;

bool isStabilizedMode(FlightMode mode) {
  return mode == STABILIZE_MODE || mode == STABLIZE_MODE_NO_I;
}

void initializeInitialControlMode() {
  currentMode = MANUAL_MODE;
  lastMode = MANUAL_MODE;
  pitchController = PitchController::PID;
  lastPitchController = PitchController::PID;
  mixerStrategy = 0.0f;
  configurationCoeff = 1.0f;
  configurationAngle = 0.0f;
  brake = false;
  changed = false;
  integralEnabled = false;
}

void prepareControlMode() {
  lastMode = currentMode;
  lastPitchController = pitchController;
  if (aircraftId() == 1) {
    // 主机：由本机遥控输入准备模式及混控配置。
    currentMode = radio_channel5_normalized > 0.2f ? MANUAL_MODE :
        (radio_channel5_normalized < -0.2f ? STABILIZE_MODE : STABLIZE_MODE_NO_I);
    mixerStrategy = radio_channel6_normalized;
    configurationCoeff = 1.0f;
    configurationAngle = radio_channel7_normalized > 0.2f ? 10.0f :
        (radio_channel7_normalized < -0.2f ? -10.0f : 0.0f);
    // 子机减父机的滚转差：上反时左侧为正、右侧为负，相邻飞机相差 10 度。
    for (unsigned i = 0; i < 6; ++i)
      targets.configuration[i] = i % 2 == 0 ? configurationAngle : -configurationAngle;
    brake = radio_channel8_normalized <= 0.2f;

    // 主机每周期根据遥控输入更新运动目标和构型目标。
    targets.throttle = 0.5f * (constrain(radio_channel3_normalized, -1.0f, 1.0f) + 1.0f);
    targets.roll = constrain(radio_channel1_normalized, -1.0f, 1.0f);
    targets.pitch = constrain(radio_channel2_normalized, -1.0f, 1.0f);
    targets.yaw = constrain(radio_channel4_normalized, -1.0f, 1.0f);
    if (currentMode != MANUAL_MODE) {
      targets.roll *= ControlParameters::maxRoll;
      targets.pitch = targets.pitch * ControlParameters::maxPitch + ControlParameters::Trim_pitch_angle;
      targets.yaw *= ControlParameters::maxYaw;
    }
  } else {
    // 从机：由所在侧命令链发布模式、俯仰目标和上级作动器贡献。
    receiveCommandData();
    AircraftCommandLink *link = aircraftIsLeft() ? CommandLinks::left : CommandLinks::right;
    if (link) slaveCommand = link->getCommandLink();
    currentMode = slaveCommand.mode;
    targets.pitch = slaveCommand.target.pitch;
    if (aircraftId() >= 2 && aircraftId() <= 7)
      slaveTargets[aircraftId() - 2] = slaveCommand.target;
  }
  displayFlightModeIndicators(currentMode);
  pitchController = ControlParameters::pitch_controller == 1
      ? PitchController::INDI : PitchController::PID;
  changed = currentMode != lastMode || pitchController != lastPitchController;
  integralEnabled = currentMode == STABILIZE_MODE;
}
