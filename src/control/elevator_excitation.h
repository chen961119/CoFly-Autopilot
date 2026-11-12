#pragma once

#include "control/control_types.h"

// Run once after mode preparation, before generating child commands.
void updateElevatorExcitation();
// Immediately discard the current sequence; a new trigger needs neutral.
void cancelElevatorExcitation();
// Physical PWM increment, applied after mapping and before actuator limits.
int16_t elevatorExcitationPwmDelta();
const ElevatorExcitationCommand &allAircraftElevatorExcitation();
