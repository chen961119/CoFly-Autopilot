#include "telemetry/telemetry.h"
#include "config/serial_config.h"
#include "parameters/parameter_service.h"

namespace {
// Extra UART buffering tolerates radio bursts between control cycles.
uint8_t receiveBuffer[1024];
uint8_t transmitBuffer[1024];
}

void beginTelemetryLink() {
  TelemetrySerial.addMemoryForRead(receiveBuffer, sizeof(receiveBuffer));
  TelemetrySerial.addMemoryForWrite(transmitBuffer, sizeof(transmitBuffer));
  TelemetrySerial.begin(TELEMETRY_BAUD, SERIAL_8N1);
}

void telemetry() {
  // Ground station polls CAPS/DATA/CONFIG and uses the existing PARAM service.
  // Do not mix the former unsolicited 55 71 binary frames into this link.
  pollTelemetryParameterService();
}
