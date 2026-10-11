#pragma once
#include <stdint.h>

// Called after SD.begin(), before controller/filter initialization.
void initializeParameterService(bool sdReady);
// Bounded serial RX/TX processing; call once at the end of a control cycle.
// SD writes are synchronous.
void pollParameterService();
// True means accepted in RAM; persistence is debounced per medium by 2 seconds.
bool saveParameterValue(uint16_t id, double value);
// Force queued writes before explicit reboot; false blocks reset.
bool flushParameterWrites();
// Latched immediately after changing any reboot-effective parameter in RAM.
bool parameterRebootRequired();
// Validate/apply the full IMU group in RAM; use ordinary delayed persistence.
// All calibration fields must reside in the same medium. True means accepted.
bool saveImuCalibrationParameters(const float *offsets);
