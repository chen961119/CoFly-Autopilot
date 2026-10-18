#include "radio_comm/radio_comm.h"
#include "control/command_generation.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "config/aircraft_config.h"
#include "control/control_modes.h"
#include "parameters/control_parameters.h"
#include "interface/human_interface.h"

FlightMode currentFlightMode = MANUAL_MODE;
FlightMode previousFlightMode = MANUAL_MODE;
PitchController activePitchController = PitchController::PID;
PitchController previousPitchController = PitchController::PID;
SlaveCommand slaveCommand{MANUAL_MODE, {}, {0, 0, 0, -0.5f, 0}};

float mixerStrategyCommand = 0.0f;
float configurationControlScale = 1.0f;
float configurationAngleDeg = 0.0f;
bool brakeEnabled = false;
bool controlModeChanged = false;

bool isStabilizedMode(FlightMode mode) {
  return mode == STABILIZE_MODE;
}

void initializeInitialControlMode() {
  currentFlightMode = MANUAL_MODE;
  previousFlightMode = MANUAL_MODE;
  activePitchController = PitchController::PID;
  previousPitchController = PitchController::PID;
  mixerStrategyCommand = 0.0f;
  configurationControlScale = 1.0f;
  configurationAngleDeg = 0.0f;
  brakeEnabled = false;
  controlModeChanged = false;
}

void prepareControlMode() {
  previousFlightMode = currentFlightMode;
  previousPitchController = activePitchController;
  if (aircraftId() == 1) {
    // 主机：由本机遥控输入准备模式及混控配置。
    currentFlightMode = radio_channel5_normalized > 0.0f ? MANUAL_MODE : STABILIZE_MODE;
    mixerStrategyCommand = 1.0f; // radio_channel6_normalized;
    configurationControlScale = 1.0f;
    configurationAngleDeg = radio_channel7_normalized > 0.2f ? 10.0f :
        (radio_channel7_normalized < -0.2f ? -10.0f : 0.0f);
    // 子机减父机的滚转差：上反时左侧为正、右侧为负，相邻飞机相差 10 度。
    for (unsigned i = 0; i < 6; ++i)
      targets.configuration[i] = i % 2 == 0 ? configurationAngleDeg : -configurationAngleDeg;
    brakeEnabled = radio_channel8_normalized <= 0.2f;

    // 主机每周期根据遥控输入更新运动目标和构型目标。
    targets.throttle = 0.5f * (constrain(radio_channel3_normalized, -1.0f, 1.0f) + 1.0f);
    targets.roll = constrain(radio_channel1_normalized, -1.0f, 1.0f);
    targets.pitch = constrain(radio_channel2_normalized, -1.0f, 1.0f);
    targets.yawRate = constrain(radio_channel4_normalized, -1.0f, 1.0f);
    if (currentFlightMode != MANUAL_MODE) {
      targets.roll *= ControlParameters::maxRoll;
      targets.pitch = targets.pitch * ControlParameters::maxPitch + ControlParameters::Trim_pitch_angle;
      targets.yawRate *= ControlParameters::maxYawRate;
    }
  } else {
    // 从机：由所在侧命令链发布模式、俯仰目标和上级作动器贡献。
    receiveCommandData();
    AircraftCommandLink *link = aircraftIsLeft() ? CommandLinks::left : CommandLinks::right;
    if (link) slaveCommand = link->getCommandLink();
    currentFlightMode = slaveCommand.mode;
    targets.pitch = slaveCommand.target.pitch;
    if (aircraftId() >= 2 && aircraftId() <= 7)
      slaveTargets[aircraftId() - 2] = slaveCommand.target;
  }
  displayFlightModeIndicators(currentFlightMode);
  activePitchController = ControlParameters::pitch_controller == 1
      ? PitchController::INDI : PitchController::PID;
  controlModeChanged = currentFlightMode != previousFlightMode || activePitchController != previousPitchController;
}
