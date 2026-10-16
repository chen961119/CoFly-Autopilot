#include "control/pitch_indi_controller.h"
#include "math/math_constants.h"
#include "control/control_output.h"
#include "parameters/control_parameters.h"
#include "system/flight_clock.h"
#include "sensors/sensor_processing.h"
#include "config/flight_config.h"
using namespace ControlParameters;

PitchIndiController::PitchIndiController() {
  gyroFiltYIndi.set_cutoff_frequency(500, 40);
  gyroFiltZIndi.set_cutoff_frequency(500, 100);
}
void PitchIndiController::reset(float pitchTarget) {
  indi_initialized = false;
  queue_head = 0;
  pitch_des_local_rate_lpf_initialized = false;
  pitch_des_local_last = pitchTarget;
  pitch_des_local_rate = 0;
}
void PitchIndiController::update(float pitchTarget, bool enabled) {
  pitchIndiControlOutput = {};
  if (!enabled) return;
  const float GyroY_raw = ang_vel[1];
  float GyroY_filt = 0.0f;
  if (fabsf(indi_pitch_pwm_to_deg_k) < 1.0e-6f ||
      fabsf(indi_pitch_effectiveness) < 1.0e-6f)
  {
    pitchIndiControlOutput.diagnostics.desiredRateDegPerSec = 0.0f;
    pitchIndiControlOutput.diagnostics.filteredRateDegPerSec = 0.0f;
    pitchIndiControlOutput.diagnostics.desiredAngularAccelerationDegPerSec2 = 0.0f;
    pitchIndiControlOutput.diagnostics.measuredAngularAccelerationDegPerSec2 = ang_acc[1];
    pitchIndiControlOutput.diagnostics.commandedElevatorDeflectionDeg = 0.0f;
    pitchIndiControlOutput.diagnostics.estimatedElevatorDeflectionDeg = eleprev;
    pitchIndiControlOutput.elevatorPwmUs = constrain((int)(PWM_CENTER_US + pwm_channel3_trim), (int)indi_pitch_pwm_min,
        (int)indi_pitch_pwm_max);
    return;
  }
  const float pwm_trim_center = PWM_CENTER_US + pwm_channel3_trim;
  const float delta_e_trim_deg =
  indi_pitch_pwm_to_deg_k * pwm_trim_center + indi_pitch_pwm_to_deg_b;
  // 进入 INDI 时，以配平点初始化独立滤波和舵机状态。
  if (!indi_initialized)
  {
    gyroFiltYIndi.reset(GyroY_raw);
    for (int i = 0; i < queue_len; i++)
    {
      delayed_pwm_queue[i] = pwm_trim_center;
    }
    delta_e_est_deg = delta_e_trim_deg;
    delta_e_cmd_prev_deg = delta_e_trim_deg;
    delta_e_cmd_lpf_prev_deg = delta_e_trim_deg;
    eleprev = delta_e_trim_deg;
    indi_initialized = true;
  }
  GyroY_filt = gyroFiltYIndi.apply(GyroY_raw);
  // 俯仰角目标的变化率前馈仍使用原限幅和低通。
  float pitch_des_local_rate_raw;
  if (dt > 1.0e-6f)
  {
    pitch_des_local_rate_raw = (pitchTarget - pitch_des_local_last) / dt;
  }
  else
  {
    pitch_des_local_rate_raw = 0.0f;
  }
  pitch_des_local_last = pitchTarget;
  pitch_des_local_rate_raw = constrain(pitch_des_local_rate_raw, -10.0, 10.0);
  if (!pitch_des_local_rate_lpf_initialized)
  {
    pitch_des_local_rate = pitch_des_local_rate_raw;
    pitch_des_local_rate_lpf_initialized = true;
  }
  else if (pitch_des_local_rate_lpf_fc > 0.0f && dt > 1.0e-6f)
  {
    float tau_pitch_des_rate =
    1.0f / ((2 * PI) * pitch_des_local_rate_lpf_fc);
    float alpha_pitch_des_rate = dt / (tau_pitch_des_rate + dt);
    pitch_des_local_rate = pitch_des_local_rate +
    alpha_pitch_des_rate * (pitch_des_local_rate_raw -
        pitch_des_local_rate);
  }
  else
  {
    pitch_des_local_rate = pitch_des_local_rate_raw;
  }
  float pitch_error_deg = pitchTarget - att[1];
  float rate_des_deg_s = pitch_error_deg * 3.0f + pitch_des_local_rate;
  float q_des_deg_s = (rate_des_deg_s + sin(att[0] / 57.3) *
      gyroFiltZIndi.apply(ang_vel[2])) /cos(att[0] / 57.3);
  q_des_deg_s = constrain(q_des_deg_s, -120, 120);
  float dq_des_deg_s2 = indi_pitch_q_gain * (q_des_deg_s - GyroY_filt);
  pitchIndiControlOutput.desiredAngularAccelerationDegPerSec2 = dq_des_deg_s2;
  pitchIndiControlOutput.desiredRateDegPerSec = q_des_deg_s;
  pitchIndiControlOutput.filteredRateDegPerSec = GyroY_filt;
  pitchIndiControlOutput.isValid = true;
  // 原增量控制律，随后进行变化率/舵偏限幅及指令滤波。
  float delta_e_indi_deg =
  0.7f * (dq_des_deg_s2 - ang_acc[1]) / indi_pitch_effectiveness;
  float delta_e_cmd_deg = delta_e_est_deg + delta_e_indi_deg;
  float max_delta_deg = indi_pitch_rate_limit_deg_s * dt;
  float delta_step_deg = delta_e_cmd_deg - delta_e_cmd_prev_deg;
  delta_step_deg = constrain(delta_step_deg, -max_delta_deg, max_delta_deg);
  delta_e_cmd_deg = delta_e_cmd_prev_deg + delta_step_deg;
  delta_e_cmd_deg = constrain(delta_e_cmd_deg, indi_pitch_defl_min,
      indi_pitch_defl_max);
  if (indi_pitch_cmd_lpf_fc_hz > 0.0f && dt > 1.0e-6f)
  {
    float tau_cmd = 1.0f / ((2 * PI) * indi_pitch_cmd_lpf_fc_hz);
    float alpha_cmd = dt / (tau_cmd + dt);
    delta_e_cmd_deg = delta_e_cmd_lpf_prev_deg +
    alpha_cmd * (delta_e_cmd_deg - delta_e_cmd_lpf_prev_deg);
  }
  delta_e_cmd_deg = constrain(delta_e_cmd_deg, indi_pitch_defl_min,
      indi_pitch_defl_max);
  delta_e_cmd_prev_deg = delta_e_cmd_deg;
  delta_e_cmd_lpf_prev_deg = delta_e_cmd_deg;
  float pwm_cmd =
  (delta_e_cmd_deg - indi_pitch_pwm_to_deg_b) / indi_pitch_pwm_to_deg_k;
  pwm_cmd = constrain(pwm_cmd, indi_pitch_pwm_min, indi_pitch_pwm_max);
  // 纯延迟队列和一阶舵机估计共用本对象的历史。
  delayed_pwm_queue[queue_head] = pwm_cmd;
  queue_head = (queue_head + 1) % queue_len;
  int delay_samples = 0;
  if (dt > 1.0e-6f)
  {
    delay_samples = (int)(indi_pitch_servo_delay_s / dt + 0.5f);
  }
  delay_samples = constrain(delay_samples, 0, queue_len - 1);
  int delayed_index = queue_head - 1 - delay_samples;
  if (delayed_index < 0)
  {
    delayed_index += queue_len;
  }
  float pwm_delayed = delayed_pwm_queue[delayed_index];
  float delta_e_delayed_deg =
  indi_pitch_pwm_to_deg_k * pwm_delayed + indi_pitch_pwm_to_deg_b;
  float tau = indi_pitch_servo_tau_s;
  if (tau < 1.0e-4f)
  {
    delta_e_est_deg = delta_e_delayed_deg;
  }
  else
  {
    float alpha_servo = dt / (tau + dt);
    delta_e_est_deg += alpha_servo * (delta_e_delayed_deg - delta_e_est_deg);
  }
  eleprev = delta_e_est_deg;
  pitchIndiControlOutput.diagnostics.desiredRateDegPerSec = q_des_deg_s;
  pitchIndiControlOutput.diagnostics.filteredRateDegPerSec = GyroY_filt;
  pitchIndiControlOutput.diagnostics.desiredAngularAccelerationDegPerSec2 = dq_des_deg_s2;
  pitchIndiControlOutput.diagnostics.measuredAngularAccelerationDegPerSec2 = ang_acc[1];
  pitchIndiControlOutput.diagnostics.commandedElevatorDeflectionDeg = delta_e_cmd_deg;
  pitchIndiControlOutput.diagnostics.estimatedElevatorDeflectionDeg = delta_e_est_deg;
  pitchIndiControlOutput.elevatorPwmUs = (int)(pwm_cmd + 0.5f);
}
