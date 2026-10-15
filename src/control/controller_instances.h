#pragma once
#include "control/roll_pid_controller.h"
#include "control/pitch_pid_controller.h"
#include "control/yaw_pid_controller.h"
#include "control/configuration_controller.h"
#include "control/pitch_indi_controller.h"
#include "control/manual_controller.h"
#include "control/roll_damping_controller.h"
#include "control/throttle_controller.h"
// 每块飞控一套本机控制器；主/从角色入口复用这些对象。
struct ControllerInstances {
  RollPidController roll;
  PitchPidController pitchPid;
  YawPidController yaw;
  ConfigurationController configuration;
  PitchIndiController pitchIndi;
  ManualController manual;
  ThrottleController throttle;
  RollDampingController rollDamping;
};
ControllerInstances &controlControllers();
