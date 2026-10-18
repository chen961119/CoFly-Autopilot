#pragma once
// Only accepted while locked. Reset waits for pending USB/X6 replies to drain.
bool requestDeviceReboot();
bool deviceRebootPending();
void pollDeviceReboot(bool responsePending);
