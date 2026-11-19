#pragma once

#include "control/control_types.h"
#include "sensors/sensor_processing.h"

// Published after the actuator write. Background readers copy one complete cycle.
struct FlightServiceSample {
  unsigned long current_time, sampledMs;
  float dt, quat[4], att[3], ang_vel[3], ang_vel_filtered[3], ang_acc[3], acc[3];
  float airspeed, relativeAngle_raw, relativeAngle_ready;
  int32_t strain_value[5];
  bool rotateValid, locked, radioValid, leftFresh, rightFresh;
  AirspeedDiagnostics airspeedStatus;
  int radioRaw[8];
  float radioNormalized[8], ActuatorOutput[5], manual_input[4];
  float wingAttitude[6][2], wingRollRate[6];
  FlightMode currentFlightMode, previousFlightMode;
  PitchController activePitchController, previousPitchController;
  float mixerStrategyCommand, configurationControlScale, configurationAngle;
  bool brakeEnabled, controlModeChanged;
  int32_t integralControlEnabled;
  ControlTargets targets;
  RollControlOutput rollControlOutput;
  PitchPidControlOutput pitchPidControlOutput;
  IndiPitchControlOutput pitchIndiControlOutput;
  RollDampingControlOutput rollDampingControlOutput;
  YawRateControlOutput yawRateControlOutput;
  ThrottleControlOutput throttleControlOutput;
  ConfigurationControlOutput configurationControlOutput;
  ActuatorCommand local;
};

void copyFlightServiceSample(FlightServiceSample &destination);
