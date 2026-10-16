#include "control/control_output.h"
#include "control/pitch_pid_controller.h"
#include "parameters/control_parameters.h"
#include "system/flight_clock.h"
#include "sensors/sensor_processing.h"
using namespace ControlParameters;

void PitchPidController::prepare(bool integralControlEnabled, bool controlModeChanged) {
  i_valid = integralControlEnabled ? 1.0f : 0.0f;
  ModeChange = controlModeChanged;
  if (controlModeChanged || !integralControlEnabled) {
    integral_pitch_prev_ol = 0;
    integral_pitch_prev_il = 0;
  }
}
void PitchPidController::update(float pitchTarget, bool enabled) {
  pitchPidControlOutput.pidOutput = 0;
  if (!enabled) return;
  float pitch_des_ol;
  error_pitch = pitchTarget - att[1];
  integral_pitch_ol = i_valid > 0.0f ? integral_pitch_prev_ol + error_pitch * dt : 0.0f;
  integral_pitch_ol =
  constrain(integral_pitch_ol, -i_limit,
      i_limit);
  if (ModeChange == 1)
  {
    integral_pitch_ol = 0;
  }
  derivative_pitch = (att[1] - previous_pitch_att) / dt;
  pitch_des_ol = Kp_pitch_angle * error_pitch +
  i_valid * Ki_pitch_angle *
  integral_pitch_ol;
  pitch_des_ol = 30.0 * pitch_des_ol;
  pitch_des_ol = (pitch_des_ol + sin(att[0] / 57.3) * ang_vel_filtered[2]) /
  constrain(cos(att[0] / 57.3), 0.5, 1);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  pitch_des_ol = (1.0 - B_loop_pitch) * pitch_des_prev + B_loop_pitch * pitch_des_ol;
  error_pitch = pitch_des_ol - ang_vel_filtered[1];
  integral_pitch_il = i_valid > 0.0f ? integral_pitch_prev_il + error_pitch * dt : 0.0f;
  integral_pitch_il =
  constrain(integral_pitch_il, -i_limit,
      i_limit);
  if (ModeChange == 1)
  {
    integral_pitch_il = 0;
  }
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID =
  .01 *
  (Kff_pitch_rate * pitch_des_ol + Kp_pitch_rate * error_pitch +
      i_valid * Ki_pitch_rate * integral_pitch_il +
      Kd_pitch_rate *
      derivative_pitch);
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  previous_pitch_att = att[1];
  pitch_des_prev = pitch_des_ol;
  pitchPidControlOutput.pidOutput = pitch_PID;
}
