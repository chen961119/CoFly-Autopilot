#include "math/units.h"
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
  configurationControlOutput.measuredAngles[0] = att_B[0] - att[0];
  configurationControlOutput.measuredAngles[1] = att_C[0] - att[0];
  configurationControlOutput.measuredAngles[2] = att_D[0] - att_B[0];
  configurationControlOutput.measuredAngles[3] = att_E[0] - att_C[0];
  configurationControlOutput.measuredAngles[4] = att_F[0] - att_D[0];
  configurationControlOutput.measuredAngles[5] = att_G[0] - att_E[0];
  configurationControlOutput.measuredRates[0] = ang_vel_B[0] - ang_vel[0];
  configurationControlOutput.measuredRates[1] = ang_vel_C[0] - ang_vel[0];
  configurationControlOutput.measuredRates[2] = ang_vel_D[0] - ang_vel_B[0];
  configurationControlOutput.measuredRates[3] = ang_vel_E[0] - ang_vel_C[0];
  configurationControlOutput.measuredRates[4] = ang_vel_F[0] - ang_vel_D[0];
  configurationControlOutput.measuredRates[5] = ang_vel_G[0] - ang_vel_E[0];

  // 非闭环周期不更新积分历史；保留最近的构型误差供主 mixer 使用。
  for (float &command : configurationControlOutput.pidOutputs) command = 0;
  for (float &rateTarget : configurationControlOutput.rateTargets) rateTarget = 0;
  if (aircraftId() != 1 || aircraftIsSingle() || !isStabilizedMode(currentFlightMode)) return;

  configurationControlOutput.angleErrors[0] = targets[0] - configurationControlOutput.measuredAngles[0];
  Phiab_des_ol = Kp_Flap * configurationControlOutput.angleErrors[0];
  configurationControlOutput.angleErrors[1] = targets[1] - configurationControlOutput.measuredAngles[1];
  Phiac_des_ol = Kp_Flap * configurationControlOutput.angleErrors[1];
  configurationControlOutput.angleErrors[2] = targets[2] - configurationControlOutput.measuredAngles[2];
  Phibd_des_ol = Kp_Flap * configurationControlOutput.angleErrors[2];
  configurationControlOutput.angleErrors[3] = targets[3] - configurationControlOutput.measuredAngles[3];
  Phice_des_ol = Kp_Flap * configurationControlOutput.angleErrors[3];
  configurationControlOutput.angleErrors[4] = targets[4] - configurationControlOutput.measuredAngles[4];
  Phidf_des_ol = Kp_Flap * configurationControlOutput.angleErrors[4];
  configurationControlOutput.angleErrors[5] = targets[5] - configurationControlOutput.measuredAngles[5];
  Phieg_des_ol = Kp_Flap * configurationControlOutput.angleErrors[5];

  Phiab_des_ol = constrain(Phiab_des_ol, -Units::toRadians(240.0f), Units::toRadians(240.0f));
  Phiac_des_ol = constrain(Phiac_des_ol, -Units::toRadians(240.0f), Units::toRadians(240.0f));
  Phibd_des_ol = constrain(Phibd_des_ol, -Units::toRadians(240.0f), Units::toRadians(240.0f));
  Phice_des_ol = constrain(Phice_des_ol, -Units::toRadians(240.0f), Units::toRadians(240.0f));
  Phidf_des_ol = constrain(Phidf_des_ol, -Units::toRadians(240.0f), Units::toRadians(240.0f));
  Phieg_des_ol = constrain(Phieg_des_ol, -Units::toRadians(240.0f), Units::toRadians(240.0f));

  configurationControlOutput.rateTargets[0] = Phiab_des_ol;
  configurationControlOutput.rateTargets[1] = Phiac_des_ol;
  configurationControlOutput.rateTargets[2] = Phibd_des_ol;
  configurationControlOutput.rateTargets[3] = Phice_des_ol;
  configurationControlOutput.rateTargets[4] = Phidf_des_ol;
  configurationControlOutput.rateTargets[5] = Phieg_des_ol;

  error_Phiab_RATE = Phiab_des_ol - configurationControlOutput.measuredRates[0];

  integral_Phiab_RATE_il = i_valid > 0.0f ? integral_Phiab_RATE_prev_il + error_Phiab_RATE * dt : 0.0f;
  integral_Phiab_RATE_il = constrain(integral_Phiab_RATE_il, -i_limit_FLAP_RATE, i_limit_FLAP_RATE);

  configurationControlOutput.pidOutputs[0] = configurationControlScale *
  (Kff_FLAP_RATE * Phiab_des_ol + Kp_FLAP_RATE * error_Phiab_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phiab_RATE_il);
  error_Phiac_RATE = Phiac_des_ol - configurationControlOutput.measuredRates[1];
  integral_Phiac_RATE_il = i_valid > 0.0f ? integral_Phiac_RATE_prev_il + error_Phiac_RATE * dt : 0.0f;
  integral_Phiac_RATE_il =
  constrain(integral_Phiac_RATE_il, -i_limit_FLAP_RATE,
      i_limit_FLAP_RATE);
  configurationControlOutput.pidOutputs[1] = configurationControlScale *
  (Kff_FLAP_RATE * Phiac_des_ol + Kp_FLAP_RATE * error_Phiac_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phiac_RATE_il);
  error_Phibd_RATE = Phibd_des_ol - configurationControlOutput.measuredRates[2];
  integral_Phibd_RATE_il = i_valid > 0.0f ? integral_Phibd_RATE_prev_il + error_Phibd_RATE * dt : 0.0f;
  integral_Phibd_RATE_il =
  constrain(integral_Phibd_RATE_il, -i_limit_FLAP_RATE,
      i_limit_FLAP_RATE);
  configurationControlOutput.pidOutputs[2] = configurationControlScale *
  (Kff_FLAP_RATE * Phibd_des_ol + Kp_FLAP_RATE * error_Phibd_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phibd_RATE_il);
  error_Phice_RATE = Phice_des_ol - configurationControlOutput.measuredRates[3];
  integral_Phice_RATE_il = i_valid > 0.0f ? integral_Phice_RATE_prev_il + error_Phice_RATE * dt : 0.0f;
  integral_Phice_RATE_il =
  constrain(integral_Phice_RATE_il, -i_limit_FLAP_RATE,
      i_limit_FLAP_RATE);
  configurationControlOutput.pidOutputs[3] = configurationControlScale *
  (Kff_FLAP_RATE * Phice_des_ol + Kp_FLAP_RATE * error_Phice_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phice_RATE_il);
  error_Phidf_RATE = Phidf_des_ol - configurationControlOutput.measuredRates[4];
  integral_Phidf_RATE_il = i_valid > 0.0f ? integral_Phidf_RATE_prev_il + error_Phidf_RATE * dt : 0.0f;
  integral_Phidf_RATE_il =
  constrain(integral_Phidf_RATE_il, -i_limit_FLAP_RATE,
      i_limit_FLAP_RATE);
  configurationControlOutput.pidOutputs[4] = configurationControlScale *
  (Kff_FLAP_RATE * Phidf_des_ol + Kp_FLAP_RATE * error_Phidf_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phidf_RATE_il);
  error_Phieg_RATE = Phieg_des_ol - configurationControlOutput.measuredRates[5];
  integral_Phieg_RATE_il = i_valid > 0.0f ? integral_Phieg_RATE_prev_il + error_Phieg_RATE * dt : 0.0f;
  integral_Phieg_RATE_il =
  constrain(integral_Phieg_RATE_il, -i_limit_FLAP_RATE,
      i_limit_FLAP_RATE);
  configurationControlOutput.pidOutputs[5] = configurationControlScale *
  (Kff_FLAP_RATE * Phieg_des_ol + Kp_FLAP_RATE * error_Phieg_RATE +
      i_valid * Ki_FLAP_RATE *
      integral_Phieg_RATE_il);

  integral_Phiab_RATE_prev_il = integral_Phiab_RATE_il;
  integral_Phiac_RATE_prev_il = integral_Phiac_RATE_il;
  integral_Phibd_RATE_prev_il = integral_Phibd_RATE_il;
  integral_Phice_RATE_prev_il = integral_Phice_RATE_il;
  integral_Phidf_RATE_prev_il = integral_Phidf_RATE_il;
  integral_Phieg_RATE_prev_il = integral_Phieg_RATE_il;
}
