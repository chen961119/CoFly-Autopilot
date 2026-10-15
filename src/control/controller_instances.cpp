#include "control/controller_instances.h"
ControllerInstances &controlControllers() {
  static ControllerInstances controllers;
  return controllers;
}
