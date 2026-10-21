#include "math/units.h"
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
SlaveCommand slaveCommand{MANUAL_MODE, {}, {0, 0, 0, 0, 0}};

float mixerStrategyCommand = 1.0f;
float configurationControlScale = 1.0f;
float configurationAngle = 0.0f;
bool brakeEnabled = false;
bool controlModeChanged = false;
float manual_input[4]{};

bool isStabilizedMode(FlightMode mode) {
  return mode == STABILIZE_MODE;
}


void initializeInitialControlMode() {
  currentFlightMode = MANUAL_MODE;
  previousFlightMode = MANUAL_MODE;
  activePitchController = PitchController::PID;
  previousPitchController = PitchController::PID;
  mixerStrategyCommand = 1.0f;
  configurationControlScale = 1.0f;
  configurationAngle = 0.0f;
  brakeEnabled = false;
  controlModeChanged = false;
  for (float &input : manual_input) input = 0.0f;
}

void prepareControlMode() {
  previousFlightMode = currentFlightMode;
  previousPitchController = activePitchController;
  if (aircraftId() == 1) {
    // 主机：由本机遥控输入准备模式及混控配置。
    currentFlightMode = radio_channel5_normalized > 0.0f ? MANUAL_MODE : STABILIZE_MODE;
    mixerStrategyCommand = 1.0f; // radio_channel6_normalized;
    configurationControlScale = 1.0f;
    configurationAngle = Units::toRadians(radio_channel7_normalized > 0.2f ? 10.0f :
        (radio_channel7_normalized < -0.2f ? -10.0f : 0.0f));
    // 子机减父机的滚转差：上反时左侧为正、右侧为负，相邻飞机相差 10 度。
    for (unsigned i = 0; i < 6; ++i)
      targets.configuration[i] = i % 2 == 0 ? configurationAngle : -configurationAngle;
    brakeEnabled = radio_channel8_normalized <= 0.2f;

    // 遥控直通量与物理目标分开保存，切换模式不改变各字段的尺度。
    manual_input[0] = constrain(radio_channel1_normalized, -1.0f, 1.0f);
    manual_input[1] = constrain(radio_channel2_normalized, -1.0f, 1.0f);
    manual_input[2] = 0.5f * (constrain(radio_channel3_normalized, -1.0f, 1.0f) + 1.0f);
    manual_input[3] = constrain(radio_channel4_normalized, -1.0f, 1.0f);

    // 主机每周期生成同一尺度的目标；手动混控只读取 manual_input。
    targets.roll = manual_input[0] * Units::toRadians(ControlParameters::maxRoll);
    targets.pitch = Units::toRadians(manual_input[1] * ControlParameters::maxPitch + ControlParameters::Trim_pitch_angle);
    targets.yawRate = manual_input[3] * Units::toRadians(ControlParameters::maxYawRate);
    targets.throttle = manual_input[2];
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
