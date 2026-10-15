#pragma once
#include "control/control_types.h"
#include "sensors/filters.h"
class PitchIndiController {
public:
  PitchIndiController();
  void reset(float pitchTarget);
  void update(float pitchTarget, bool enabled);
private:
  static constexpr int queue_len = 64;
  bool indi_initialized = false;
  float delayed_pwm_queue[queue_len] = {};
  int queue_head = 0;
  float delta_e_est_deg = 0, delta_e_cmd_prev_deg = 0, delta_e_cmd_lpf_prev_deg = 0;
  double eleprev = 0;
  bool pitch_des_local_rate_lpf_initialized = false;
  float pitch_des_local_last = 0, pitch_des_local_rate = 0;
  PX4LowPassFilter2p gyroFiltYIndi, gyroFiltZIndi;
};
