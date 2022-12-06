#include "EventTriggerSystem.hh"

#include <gz/plugin/Register.hh>
#include <gz/plugin/RegisterMore.hh>

using namespace gz;
using namespace gz::sim;

IGNITION_ADD_PLUGIN(EventTriggerSystem,
    System,
    EventTriggerSystem::ISystemConfigure,
    EventTriggerSystem::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(EventTriggerSystem, "gz::sim::EventTriggerSystem")
