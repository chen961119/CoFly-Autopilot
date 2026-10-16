#pragma once

// SD 日志初始化、状态查询和周期记录接口。
void initializeLogger();
bool loggerSdReady();
void flushLogger();
unsigned int loggerFileNumber();
// 所有飞机共用固定 CSV 字段及顺序，按日志频率记录本机状态。
void logFlightData();
