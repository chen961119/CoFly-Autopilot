#include "control/control_pipeline.h"
#include "control/control_output.h"
#include "control/command_generation.h"
#include "control/master_controller.h"
#include "control/slave_controller.h"
#include "control/control_modes.h"
#include "allocation/master_mixer.h"
#include "allocation/slave_mixer.h"
#include "allocation/command_fusion.h"
#include "config/aircraft_config.h"
#include "interaircraft_comm/interaircraft_comm.h"

ControlCycleOutput commands{};

void runControlCycle() {
  commands = {};
  prepareControlMode();
  if (aircraftId() == 1) {
    runMasterController();
    mixMasterCommands();
    generateSlaveCommand();
  } else {
    runSlaveController();
    mixSlaveCommands();
    fuseSlaveCommands();
  }
}
