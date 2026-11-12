#include "control/elevator_excitation.h"
#include "control/elevator_excitation_state.h"
#include "control/control_modes.h"
#include "config/aircraft_config.h"
#include "parameters/control_parameters.h"
#include "radio_comm/radio_comm.h"
#include "system/flight_lock.h"
#include <Arduino.h>
#include <cstring>

namespace {
ElevatorExcitation::Master master;
ElevatorExcitation::Follower follower;
int16_t localDelta = 0;

bool validAuxiliary(int raw, float normalized) {
  uint32_t bits;
  std::memcpy(&bits, &normalized, sizeof(bits));
  return raw >= 800 && raw <= 2200 && (bits & 0x7f800000u) != 0x7f800000u &&
         normalized >= -1.0f && normalized <= 1.0f;
}
} // namespace

void updateElevatorExcitation() {
  const uint32_t now = millis();
  const bool enabled = currentFlightMode == MANUAL_MODE && !isFlightLocked();
  if (aircraftId() == 1) {
    using namespace ControlParameters;
    uint32_t durationBits;
    std::memcpy(&durationBits, &elev_step_t_s, sizeof(durationBits));
    const bool parametersValid = (durationBits & 0x7f800000u) != 0x7f800000u &&
        elev_step_t_s >= 0.01f && elev_step_t_s <= 60.0f &&
        elev_step_x_us >= -500 && elev_step_x_us <= 500 &&
        elev_step_y_us >= -500 && elev_step_y_us <= 500;
    const uint16_t duration = parametersValid ? static_cast<uint16_t>(elev_step_t_s * 1000.0f + 0.5f) : 0;
    // Current SBUS provides all auxiliary channels in the same receiver frame.
    // PWM/PPM has no CH7 and no auxiliary freshness tracking; fail closed there.
    bool receiverReady = false;
#if defined USE_SBUS_RX
    receiverReady = radioInputValid() && validAuxiliary(radio_channel5_raw, radio_channel5_normalized);
#endif
    master.update(now, enabled && receiverReady && parametersValid,
        validAuxiliary(radio_channel6_raw, radio_channel6_normalized), radio_channel6_normalized,
        validAuxiliary(radio_channel7_raw, radio_channel7_normalized), radio_channel7_normalized,
        parametersValid ? elev_step_x_us : 0, parametersValid ? elev_step_y_us : 0, duration);
    localDelta = master.localDelta();
  } else {
    localDelta = follower.update(now, enabled, slaveCommand.elevatorExcitation);
  }
}

void cancelElevatorExcitation() {
  master.cancel();
  follower.cancel();
  localDelta = 0;
}

int16_t elevatorExcitationPwmDelta() { return localDelta; }

const ElevatorExcitationCommand &allAircraftElevatorExcitation() { return master.command(); }
