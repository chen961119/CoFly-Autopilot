#include "math/units.h"
#include "control/control_output.h"
#include "control/yaw_rate_pid_controller.h"
#include "sensors/sensor_processing.h"
#include <Arduino.h>
#include "parameters/control_parameters.h"
#include "system/flight_clock.h"
using namespace ControlParameters;

void YawRatePidController::prepare(bool controlModeChanged)
{
  if (controlModeChanged)
  {
    integral_yaw_rate_prev = 0;
    integral_yaw_rate = 0;
    derivativeInitialized = false;
  }
}
void YawRatePidController::update(float yawRateTarget, float rollTarget, float pitchTarget, bool enabled)
{
  yawRateControlOutput = {};
  if (!enabled) {
    derivativeInitialized = false;
    return;
  }
  // 协调转弯前馈与遥控偏航角速度目标相加，再与机体 Z 轴角速度反馈比较，单位均为 rad/s。
  const float coordinatedRate = Units::acceleration(1.0f) / V_cruise * tan(rollTarget) * cos(pitchTarget);
  error_yaw_rate = yawRateTarget + coordinatedRate - ang_vel_filtered[2];
  integral_yaw_rate = Ki_yaw_rate != 0.0f
      ? integral_yaw_rate_prev + Ki_yaw_rate * error_yaw_rate * dt : 0.0f;
  integral_yaw_rate =
      constrain(integral_yaw_rate, -i_limit_yaw_rate,
                i_limit_yaw_rate);
  // 首次启用或模式切换后先建立差分起点，避免跨模式的旧误差产生 D 冲击。
  derivative_yaw_rate = derivativeInitialized && Kd_yaw_rate != 0.0f && dt > 0.0f
      ? Kd_yaw_rate * (error_yaw_rate - error_yaw_rate_prev) / dt : 0.0f;
  derivativeInitialized = true;
  proportion_yaw_rate = Kp_yaw_rate * error_yaw_rate;
  feedforward_yaw_rate = Kff_yaw_rate * (yawRateTarget + coordinatedRate);

  yaw_rate_PID = feedforward_yaw_rate + proportion_yaw_rate +  integral_yaw_rate + derivative_yaw_rate;
  error_yaw_rate_prev = error_yaw_rate;
  integral_yaw_rate_prev = integral_yaw_rate;
  yawRateControlOutput.pidOutput = yaw_rate_PID;
  yawRateControlOutput.proportionalTerm = proportion_yaw_rate;
  yawRateControlOutput.integralTerm = integral_yaw_rate;
  yawRateControlOutput.derivativeTerm = derivative_yaw_rate;
  yawRateControlOutput.feedforwardTerm = feedforward_yaw_rate;
  yawRateControlOutput.rateTarget = yawRateTarget + coordinatedRate;
}
