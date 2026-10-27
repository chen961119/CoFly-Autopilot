#pragma once
#include <stddef.h>

// Shares the parameter service's bounded USB/X6 RX/TX; never read a port here.
void handleGroundStationCommand(char *line, char *response, size_t capacity);

// nullptr means accepted; otherwise returns the protocol error code.
const char *requestGroundCalibration(const char *sensor);
const char *requestGroundSensorSetting(const char *sensor, const char *setting);
