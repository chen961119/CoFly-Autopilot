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
  const bool unlocked = !isFlightLocked();
  const bool manual = currentFlightMode == MANUAL_MODE;
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
    RadioInputStatus receiver = RadioInputStatus::Invalid;
#if defined USE_SBUS_RX
    receiver = radioInputStatus();
#endif
    const bool manualInput = manual && validAuxiliary(radio_channel5_raw, radio_channel5_normalized);
    const bool centerValid = validAuxiliary(radio_channel6_raw, radio_channel6_normalized);
    const bool allValid = validAuxiliary(radio_channel7_raw, radio_channel7_normalized);
    const bool ready = unlocked && parametersValid;
    if (ready && receiver != RadioInputStatus::Valid) {
      // Failsafe may supply preset CH5/6/7 positions. Only a good frame can
      // establish a new switch position or confirm that manual mode was left.
      const bool keepPulse = receiver == RadioInputStatus::LostFrame && manualInput;
      master.skipSwitchSample(now, keepPulse && centerValid, keepPulse && allValid);
    } else {
      master.update(now, ready && manualInput, centerValid, radio_channel6_normalized,
          allValid, radio_channel7_normalized, parametersValid ? elev_step_x_us : 0,
          parametersValid ? elev_step_y_us : 0, duration);
    }
    localDelta = master.localDelta();
  } else {
    localDelta = follower.update(now, unlocked, slaveCommand.elevatorExcitation, manual);
  }
}

void cancelElevatorExcitation() {
  master.cancel();
  follower.cancel();
  localDelta = 0;
}

int16_t elevatorExcitationPwmDelta() { return localDelta; }

const ElevatorExcitationCommand &allAircraftElevatorExcitation() { return master.command(); }
