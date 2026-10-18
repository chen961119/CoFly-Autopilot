#pragma once
#include <stddef.h>

// Shares the parameter service's bounded USB/X6 RX/TX; never read a port here.
void handleGroundStationCommand(char *line, char *response, size_t capacity);

enum class GroundStationResult { NotImplemented };
// V1 contracts only. Implement asynchronous jobs/progress in the next version.
GroundStationResult requestGroundCalibration(const char *sensor);
GroundStationResult requestGroundSensorSetting(const char *sensor, const char *setting);
