#include "control/configuration_controller.h"
#include "control/control_output.h"
#include "config/aircraft_config.h"
#include "control/control_modes.h"
#include "radio_comm/radio_comm.h"
#include "parameters/control_parameters.h"
#include "system/flight_clock.h"
#include "sensors/sensor_processing.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "config/flight_config.h"
using namespace ControlParameters;

void ConfigurationController::update(const float (&targets)[6]) {
  configurationControlOutput.angles[0] = att_B[0] - att[0];
  configurationControlOutput.angles[1] = att_C[0] - att[0];
  configurationControlOutput.angles[2] = att_D[0] - att_B[0];
  configurationControlOutput.angles[3] = att_E[0] - att_C[0];
  configurationControlOutput.angles[4] = att_F[0] - att_D[0];
  configurationControlOutput.angles[5] = att_G[0] - att_E[0];
  configurationControlOutput.rates[0] = ang_vel_B[0] - (ang_vel[0]);
  configurationControlOutput.rates[1] = ang_vel_C[0] - (ang_vel[0]);
  configurationControlOutput.rates[2] = ang_vel_D[0] - ang_vel_B[0];
  configurationControlOutput.rates[3] = ang_vel_E[0] - ang_vel_C[0];
  configurationControlOutput.rates[4] = ang_vel_F[0] - ang_vel_D[0];
  configurationControlOutput.rates[5] = ang_vel_G[0] - ang_vel_E[0];
  // 非闭环周期不更新积分历史；保留最近的构型误差供主 mixer 使用。
  for (float &command : configurationControlOutput.commands) command = 0;
  if (aircraftId() != 1 || aircraftIsSingle() || !isStabilizedMode(currentMode)) return;

  configurationControlOutput.errors[0] = targets[0] - configurationControlOutput.angles[0];
  Phiab_des_ol =
  Kp_Flap * configurationControlOutput.errors[0];
  configurationControlOutput.errors[1] = targets[1] - configurationControlOutput.angles[1];
  Phiac_des_ol = Kp_Flap * configurationControlOutput.errors[1];
  configurationControlOutput.errors[2] = targets[2] - configurationControlOutput.angles[2];
  Phibd_des_ol = Kp_Flap * configurationControlOutput.errors[2];
  configurationControlOutput.errors[3] = targets[3] - configurationControlOutput.angles[3];
  Phice_des_ol = Kp_Flap * configurationControlOutput.errors[3];
  configurationControlOutput.errors[4] = targets[4] - configurationControlOutput.angles[4];
  Phidf_des_ol = Kp_Flap * configurationControlOutput.errors[4];
  configurationControlOutput.errors[5] = targets[5] - configurationControlOutput.angles[5];
  Phieg_des_ol = Kp_Flap * configurationControlOutput.errors[5];
  float Kl = 30.0;
  Phiab_des_ol = Kl * Phiab_des_ol;
  Phiac_des_ol = Kl * Phiac_des_ol;
  Phibd_des_ol = Kl * Phibd_des_ol;
  Phice_des_ol = Kl * Phice_des_ol;
  Phidf_des_ol = Kl * Phidf_des_ol;
  Phieg_des_ol = Kl * Phieg_des_ol;
  Phiab_des_ol = constrain(Phiab_des_ol, -240.0, 240.0);
  Phiac_des_ol = constrain(Phiac_des_ol, -240.0, 240.0);
  Phibd_des_ol = constrain(Phibd_des_ol, -240.0, 240.0);
  Phice_des_ol = constrain(Phice_des_ol, -240.0, 240.0);
  Phidf_des_ol = constrain(Phidf_des_ol, -240.0, 240.0);
  Phieg_des_ol = constrain(Phieg_des_ol, -240.0, 240.0);
  error_Phiab_RATE = Phiab_des_ol - configurationControlOutput.rates[0];
  integral_Phiab_RATE_il = integral_Phiab_RATE_prev_il + error_Phiab_RATE * dt;
  integral_Phiab_RATE_il =
  constrain(integral_Phiab_RATE_il, -i_limit,
      i_limit);
  configurationControlOutput.commands[0] = coeffconfiguration * .01 *
  (Kff_FLAP_RATE * Phiab_des_ol + Kp_FLAP_RATE * error_Phiab_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phiab_RATE_il);
  error_Phiac_RATE = Phiac_des_ol - configurationControlOutput.rates[1];
  integral_Phiac_RATE_il = integral_Phiac_RATE_prev_il + error_Phiac_RATE * dt;
  integral_Phiac_RATE_il =
  constrain(integral_Phiac_RATE_il, -i_limit,
      i_limit);
  configurationControlOutput.commands[1] = coeffconfiguration * .01 *
  (Kff_FLAP_RATE * Phiac_des_ol + Kp_FLAP_RATE * error_Phiac_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phiac_RATE_il);
  error_Phibd_RATE = Phibd_des_ol - configurationControlOutput.rates[2];
  integral_Phibd_RATE_il = integral_Phibd_RATE_prev_il + error_Phibd_RATE * dt;
  integral_Phibd_RATE_il =
  constrain(integral_Phibd_RATE_il, -i_limit,
      i_limit);
  configurationControlOutput.commands[2] = coeffconfiguration * .01 *
  (Kff_FLAP_RATE * Phibd_des_ol + Kp_FLAP_RATE * error_Phibd_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phibd_RATE_il);
  error_Phice_RATE = Phice_des_ol - configurationControlOutput.rates[3];
  integral_Phice_RATE_il = integral_Phice_RATE_prev_il + error_Phice_RATE * dt;
  integral_Phice_RATE_il =
  constrain(integral_Phice_RATE_il, -i_limit,
      i_limit);
  configurationControlOutput.commands[3] = coeffconfiguration * .01 *
  (Kff_FLAP_RATE * Phice_des_ol + Kp_FLAP_RATE * error_Phice_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phice_RATE_il);
  error_Phidf_RATE = Phidf_des_ol - configurationControlOutput.rates[4];
  integral_Phidf_RATE_il = integral_Phidf_RATE_prev_il + error_Phidf_RATE * dt;
  integral_Phidf_RATE_il =
  constrain(integral_Phidf_RATE_il, -i_limit,
      i_limit);
  configurationControlOutput.commands[4] = coeffconfiguration * .01 *
  (Kff_FLAP_RATE * Phidf_des_ol + Kp_FLAP_RATE * error_Phidf_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phidf_RATE_il);
  error_Phieg_RATE = Phieg_des_ol - configurationControlOutput.rates[5];
  integral_Phieg_RATE_il = integral_Phieg_RATE_prev_il + error_Phieg_RATE * dt;
  integral_Phieg_RATE_il =
  constrain(integral_Phieg_RATE_il, -i_limit,
      i_limit);
  configurationControlOutput.commands[5] = coeffconfiguration * .01 *
  (Kff_FLAP_RATE * Phieg_des_ol + Kp_FLAP_RATE * error_Phieg_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phieg_RATE_il);
  Phiab_des_prev = Phiab_des_ol;
  Phiac_des_prev = Phiac_des_ol;
  Phibd_des_prev = Phibd_des_ol;
  Phice_des_prev = Phice_des_ol;
  Phidf_des_prev = Phidf_des_ol;
  Phieg_des_prev = Phieg_des_ol;
  integral_Phiab_RATE_prev_il = integral_Phiab_RATE_il;
  integral_Phiac_RATE_prev_il = integral_Phiac_RATE_il;
  integral_Phibd_RATE_prev_il = integral_Phibd_RATE_il;
  integral_Phice_RATE_prev_il = integral_Phice_RATE_il;
  integral_Phidf_RATE_prev_il = integral_Phidf_RATE_il;
  integral_Phieg_RATE_prev_il = integral_Phieg_RATE_il;
}
