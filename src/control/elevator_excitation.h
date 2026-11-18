#pragma once

#include "control/control_types.h"

// Run once after mode preparation, before generating child commands.
void updateElevatorExcitation();
// Immediately discard the current sequence; a new trigger needs neutral.
void cancelElevatorExcitation();
// Elevator increment in microseconds, positive down. Each board applies its own REV.
int16_t elevatorExcitationPwmDelta();
const ElevatorExcitationCommand &allAircraftElevatorExcitation();
