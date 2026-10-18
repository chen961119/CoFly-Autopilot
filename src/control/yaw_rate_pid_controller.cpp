#include "math/units.h"
#include "control/control_output.h"
#include "control/yaw_rate_pid_controller.h"
#include "sensors/sensor_processing.h"
#include <Arduino.h>
#include "parameters/control_parameters.h"
#include "system/flight_clock.h"
using namespace ControlParameters;

void YawRatePidController::prepare(bool integralControlEnabled, bool controlModeChanged)
{
  i_valid = integralControlEnabled ? 1.0f : 0.0f;
  ModeChange = controlModeChanged;
  if (controlModeChanged || !integralControlEnabled)
  {
    integral_yaw_rate_prev = 0;
  }
}
void YawRatePidController::update(float yawRateTarget, float rollTarget, float pitchTarget, bool enabled)
{
  yawRateControlOutput.pidOutput = 0;
  if (!enabled)
    return;
  // 协调转弯前馈与遥控偏航角速度目标相加，再与机体 Z 轴角速度反馈比较，单位均为 deg/s。
  const float coordinatedRate = 57.3 * (9.8 / V_cruise * tan(rollTarget / 57.3) * cos(pitchTarget / 57.3));
  error_yaw_rate = yawRateTarget + coordinatedRate - Units::toDegrees(ang_vel_filtered[2]);
  integral_yaw_rate = i_valid > 0.0f ? integral_yaw_rate_prev + error_yaw_rate * dt : 0.0f;
  integral_yaw_rate =
      constrain(integral_yaw_rate, -i_limit,
                i_limit);
  if (ModeChange == 1)
  {
    integral_yaw_rate = 0;
  }
  derivative_yaw_rate = (error_yaw_rate - error_yaw_rate_prev) / dt;

  yaw_rate_PID = .01 * (Kff_yaw_rate * (yawRateTarget + coordinatedRate) +
       Kp_yaw_rate * error_yaw_rate + i_valid * Ki_yaw_rate * integral_yaw_rate +
       Kd_yaw_rate * derivative_yaw_rate);
  error_yaw_rate_prev = error_yaw_rate;
  integral_yaw_rate_prev = integral_yaw_rate;
  yawRateControlOutput.pidOutput = yaw_rate_PID;
}
