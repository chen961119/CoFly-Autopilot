#pragma once

// CUAV P9 transparent UART: initialize once, poll once per control cycle.
void beginTelemetryLink();
void telemetry();
