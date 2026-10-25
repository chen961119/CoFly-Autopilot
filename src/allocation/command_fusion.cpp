#include "allocation/command_fusion.h"
#include "control/control_pipeline.h"
#include "control/control_modes.h"

void fuseSlaveCommands() {
  // 上级分配与本机贡献均已完成各自的尺度转换，只相加，禁止重复缩放。
  commands.local.aileron1 += slaveCommand.masterContribution.aileron1;
  commands.local.aileron2 += slaveCommand.masterContribution.aileron2;
  commands.local.elevator += slaveCommand.masterContribution.elevator;
  commands.local.throttle += slaveCommand.masterContribution.throttle;
  commands.local.rudder += slaveCommand.masterContribution.rudder;
}
