#include "system/device_reboot.h"
#include "system/flight_lock.h"
#include "logging/logger.h"
#include "config/serial_config.h"
#include "parameters/parameter_service.h"
#include <Arduino.h>

namespace {
bool pending = false;
bool waiting = false;
uint32_t replyTime = 0;
}

bool deviceRebootPending() { return pending; }
bool requestDeviceReboot() {
  if (!isFlightLocked()) return false;
  if (!pending && !flushParameterWrites()) return false;
  if (!pending) { pending = true; waiting = false; }
  setFlightLocked(true);
  return true;
}

void pollDeviceReboot(bool responsePending) {
  if (!pending) return;
  if (responsePending) { waiting = false; return; }
  if (!waiting) { replyTime = millis(); waiting = true; return; }
  if (static_cast<uint32_t>(millis() - replyTime) < 250) return;
  USBSerial.flush();
  flushLogger();
  __asm__ volatile("dsb" ::: "memory");
  SCB_AIRCR = 0x05FA0004; // Cortex-M7 SYSRESETREQ, no bootloader entry.
  __asm__ volatile("dsb" ::: "memory");
  while (true) {} // Hardware reset follows immediately.
}
