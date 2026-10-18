#pragma once

// 3DR X6 transparent UART: initialize once, poll once per control cycle.
void beginTelemetryLink();
void telemetry();
