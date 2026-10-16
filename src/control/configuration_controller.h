#pragma once
#include "control/control_types.h"
class ConfigurationController {
public:
  void prepare(bool integralControlEnabled) {
    i_valid = integralControlEnabled ? 1.0f : 0.0f;
    if (!integralControlEnabled) {
      integral_Phiab_RATE_prev_il = integral_Phiac_RATE_prev_il = 0;
      integral_Phibd_RATE_prev_il = integral_Phice_RATE_prev_il = 0;
      integral_Phidf_RATE_prev_il = integral_Phieg_RATE_prev_il = 0;
    }
  }
  // 始终刷新构型测量，仅主机增稳模式运行构型闭环并发布控制量。
  void update(const float (&targets)[6]);
private:
  float i_valid = 0;
  float Phiab_des_ol = 0;
  float Phiab_des_prev = 0;
  float error_Phiab_RATE = 0;
  float integral_Phiab_RATE_il = 0;
  float integral_Phiab_RATE_prev_il = 0;
  float Phiac_des_ol = 0;
  float Phiac_des_prev = 0;
  float error_Phiac_RATE = 0;
  float integral_Phiac_RATE_il = 0;
  float integral_Phiac_RATE_prev_il = 0;
  float Phibd_des_ol = 0;
  float Phibd_des_prev = 0;
  float error_Phibd_RATE = 0;
  float integral_Phibd_RATE_il = 0;
  float integral_Phibd_RATE_prev_il = 0;
  float Phice_des_ol = 0;
  float Phice_des_prev = 0;
  float error_Phice_RATE = 0;
  float integral_Phice_RATE_il = 0;
  float integral_Phice_RATE_prev_il = 0;
  float Phidf_des_ol = 0;
  float Phidf_des_prev = 0;
  float error_Phidf_RATE = 0;
  float integral_Phidf_RATE_il = 0;
  float integral_Phidf_RATE_prev_il = 0;
  float Phieg_des_ol = 0;
  float Phieg_des_prev = 0;
  float error_Phieg_RATE = 0;
  float integral_Phieg_RATE_il = 0;
  float integral_Phieg_RATE_prev_il = 0;
};
