#pragma once
#include "control/control_types.h"
class ManualController {
public:
  void update(const MasterControlTargets &targets) const;
};
