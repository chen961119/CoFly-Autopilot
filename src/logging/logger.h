#pragma once
#include <stdint.h>

// SD 日志初始化、状态查询和周期记录接口。
void initializeLogger();
bool loggerSdReady();
void flushLogger();
unsigned int loggerFileNumber();
void startFlightStorage();
// Control producer and low-priority storage consumer, respectively.
void captureFlightServices(bool logEnabled);
void serviceFlightLogger();
uint32_t droppedFlightLogSamples();
uint32_t flightLogWriteFailures();
