#include "control/control_output.h"
#include "control/yaw_pid_controller.h"
#include "sensors/sensor_processing.h"
#include <Arduino.h>
#include "parameters/control_parameters.h"
#include "system/flight_clock.h"
using namespace ControlParameters;

void YawPidController::prepare(bool integralControlEnabled, bool controlModeChanged)
{
  i_valid = integralControlEnabled ? 1.0f : 0.0f;
  ModeChange = controlModeChanged;
  if (controlModeChanged || !integralControlEnabled)
  {
    integral_yaw_prev = 0;
  }
}
void YawPidController::update(float yawTarget, float rollTarget, float pitchTarget, bool enabled)
{
  yawControlOutput.pidOutput = 0;
  if (!enabled)
    return;
  const float coordinatedRate = 57.3 * (9.8 / V_cruise * tan(rollTarget / 57.3) * cos(pitchTarget / 57.3));
  error_yaw = yawTarget + coordinatedRate - ang_vel_filtered[2];
  integral_yaw = i_valid > 0.0f ? integral_yaw_prev + error_yaw * dt : 0.0f;
  integral_yaw =
      constrain(integral_yaw, -i_limit,
                i_limit);
  if (ModeChange == 1)
  {
    integral_yaw = 0;
  }
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;

  yaw_PID = .01 * (Kff_yaw_rate * (yawTarget + coordinatedRate) +
       Kp_yaw * error_yaw + i_valid * Ki_yaw * integral_yaw +
       Kd_yaw * derivative_yaw);
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
  yawControlOutput.pidOutput = yaw_PID;
}
