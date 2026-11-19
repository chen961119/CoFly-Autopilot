#pragma once
#include <stddef.h>
#include <stdint.h>

// Called after SD.begin(), before controller/filter initialization.
void initializeParameterService(bool sdReady);
// Called by the parameter task for USB RX/TX, deferred saves and reboot.
// The storage task acknowledges durable operations before the protocol reply.
void pollParameterService();
// Same protocol on the X6 UART, with independent RX/TX and listing state.
void pollTelemetryParameterService();
// Best-effort complete diagnostic frame on each idle USB/X6 session.
// Busy links drop independently; poll drains accepted frames without blocking.
bool queueDiagnosticOutput(const char *text, size_t length);
bool queueUsbDiagnosticOutput(const char *text, size_t length);
// Keep locked-only, durable transactions locked until verified or rejected.
bool parameterMaintenanceBusy();
class LockedParameterMaintenance {
public:
  LockedParameterMaintenance();
  ~LockedParameterMaintenance();
  explicit operator bool() const { return acquired; }
  LockedParameterMaintenance(const LockedParameterMaintenance &) = delete;
  LockedParameterMaintenance &operator=(const LockedParameterMaintenance &) = delete;
private:
  bool acquired;
};
// True means accepted in RAM; persistence is debounced per medium by 2 seconds.
// EEPROM edits require lock; SD edits and saves are allowed locked or unlocked.
bool saveParameterValue(const char *name, double value);
// Force queued writes before explicit reboot; false blocks reset.
// Both media can be flushed regardless of flight lock state.
bool flushParameterWrites();
bool parameterWritesPending();
// Latched immediately after changing any reboot-effective parameter in RAM.
bool parameterRebootRequired();
// Validate/apply the full IMU group in RAM; use ordinary delayed persistence.
// All calibration fields must reside in the same medium. True means accepted.
bool saveImuCalibrationParameters(const float *offsets);
// Parse/validate all CH1-4 MIN/MAX/TRIM/REV values; persist one SD snapshot
// before applying to RAM. Failure leaves the live radio parameters unchanged.
bool saveRadioCalibrationParameters(const char *const *values);
