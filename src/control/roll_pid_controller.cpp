#include "math/units.h"
#include "control/control_output.h"
#include "control/roll_pid_controller.h"
#include "math/math_constants.h"
#include "config/aircraft_config.h"
#include "parameters/control_parameters.h"
#include "system/flight_clock.h"
#include "sensors/sensor_processing.h"
#include "interaircraft_comm/interaircraft_comm.h"
using namespace ControlParameters;

void RollPidController::prepare(bool integralControlEnabled, bool controlModeChanged) {
  i_valid = integralControlEnabled ? 1.0f : 0.0f;
  ModeChange = controlModeChanged;
  if (controlModeChanged || !integralControlEnabled) {
    integral_roll_prev_ol = 0;
    integral_roll_prev_il = 0;
  }
}
void RollPidController::update(float rollTarget, bool enabled) {
  rollControlOutput = {0, roll_PID_lpf, roll_PID_dot_lpf, roll_eq};
  if (!enabled) return;
  float roll_des_ol = 0;
  if (aircraftIsSingle())
  {
    roll_eq = att[0];
  }
  else
  {
    float phiA, phiB, phiC, phiD, phiE, phiF, phiG;
    phiA = att[0];
    phiB = att_B[0];
    phiC = att_C[0];
    phiD = att_D[0];
    phiE = att_E[0];
    phiF = att_F[0];
    phiG = att_G[0];
    if ((aircraftCount() == 3))
    {
      roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC)) /
          (cos(phiA) + cos(phiB) + cos(phiC)));
    }
    else if ((aircraftCount() == 4))
    {
      roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC) + sin(phiD)) /
          (cos(phiA) + cos(phiB) + cos(phiC) + cos(phiD)));
    }
    else if ((aircraftCount() == 5))
    {
      roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC) + sin(phiD) + sin(phiE)) /
          (cos(phiA) + cos(phiB) + cos(phiC) + cos(phiD) + cos(phiE)));
    }
    else if ((aircraftCount() == 7))
    {
      roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC) + sin(phiD) + sin(phiE) +
          sin(phiF) + sin(phiG)) /
          (cos(phiA) + cos(phiB) + cos(phiC) + cos(phiD) + cos(phiE) +
          cos(phiF) + cos(phiG)));
    }
  }
  error_roll = rollTarget - roll_eq;
  integral_roll_ol = i_valid > 0.0f ? integral_roll_prev_ol + error_roll * dt : 0.0f;
  integral_roll_ol =
  constrain(integral_roll_ol, -i_limit_roll_angle,
      i_limit_roll_angle);
  if (ModeChange == 1)
  {
    integral_roll_ol = 0;
  }
  derivative_roll = (att[0] - previous_roll_att) / dt;
  roll_des_ol = Kp_roll_angle * error_roll +
  i_valid * Ki_roll_angle *
  integral_roll_ol;
  roll_des_ol = constrain(roll_des_ol, -Units::toRadians(240.0f), Units::toRadians(240.0f));
  roll_des_ol = (1.0 - B_loop_roll) * roll_des_prev + B_loop_roll * roll_des_ol;
  float Rollrate;
  if (aircraftIsSingle())
  {
    Rollrate = ang_vel_filtered[0];
  }
  else
  {
    if ((aircraftCount() == 3))
    {
      Rollrate = (ang_vel_B[0] + ang_vel_C[0] + ang_vel_filtered[0]) / 3.0;
    }
    else if ((aircraftCount() == 4))
    {
      Rollrate = (ang_vel_B[0] + ang_vel_C[0] + ang_vel_D[0] + ang_vel_filtered[0]) / 4.0;
    }
    else if ((aircraftCount() == 5))
    {
      Rollrate = (ang_vel_B[0] + ang_vel_C[0] + ang_vel_D[0] + ang_vel_E[0] + ang_vel_filtered[0]) / 5.0;
    }
    else if ((aircraftCount() == 7))
    {
      Rollrate = (ang_vel_B[0] + ang_vel_C[0] + ang_vel_D[0] + ang_vel_E[0] + ang_vel_F[0] + ang_vel_G[0] +
          ang_vel_filtered[0]) /
      7.0;
    }
  }
  error_roll = roll_des_ol - Rollrate;
  integral_roll_il = i_valid > 0.0f ? integral_roll_prev_il + error_roll * dt : 0.0f;
  integral_roll_il =
  constrain(integral_roll_il, -i_limit_roll_rate,
      i_limit_roll_rate);
  if (ModeChange == 1)
  {
    integral_roll_il = 0;
  }
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID =
  (Kff_roll_rate * roll_des_ol + Kp_roll_rate * error_roll +
      i_valid * Ki_roll_rate * integral_roll_il +
      Kd_roll_rate *
      derivative_roll);
  float tau_roll_pid = 1.0f / ((2 * PI) * roll_pid_lpf_fc);
  float alpha_roll_pid = dt / (tau_roll_pid + dt);
  roll_PID_lpf = roll_PID_lpf + alpha_roll_pid * (roll_PID - roll_PID_lpf);
  if (!roll_pid_dot_initialized)
  {
    roll_PID_prev = roll_PID;
    roll_PID_dot = 0.0f;
    roll_PID_dot_lpf = 0.0f;
    roll_pid_dot_initialized = true;
  }
  else
  {
    if (dt > 1.0e-6f)
    {
      roll_PID_dot = (roll_PID - roll_PID_prev) / dt;
    }
    else
    {
      roll_PID_dot = 0.0f;
    }
    if (roll_pid_dot_lpf_fc > 0.0f && dt > 1.0e-6f)
    {
      float tau_dot = 1.0f / ((2 * PI) * roll_pid_dot_lpf_fc);
      float alpha_dot = dt / (tau_dot + dt);
      roll_PID_dot_lpf =
      roll_PID_dot_lpf + alpha_dot * (roll_PID_dot - roll_PID_dot_lpf);
    }
    else
    {
      roll_PID_dot_lpf = roll_PID_dot;
    }
    roll_PID_prev = roll_PID;
  }
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  previous_roll_att = att[0];
  roll_des_prev = roll_des_ol;
  rollControlOutput = {roll_PID, roll_PID_lpf, roll_PID_dot_lpf, roll_eq};
}
