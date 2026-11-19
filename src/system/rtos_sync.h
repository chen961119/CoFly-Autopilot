#pragma once

#include <FreeRTOS.h>
#include <task.h>

// Task context only. Protect short copies on the single-core RT1062.
class TaskCriticalSection {
public:
  TaskCriticalSection() { taskENTER_CRITICAL(); }
  ~TaskCriticalSection() { taskEXIT_CRITICAL(); }
  TaskCriticalSection(const TaskCriticalSection &) = delete;
  TaskCriticalSection &operator=(const TaskCriticalSection &) = delete;
};
