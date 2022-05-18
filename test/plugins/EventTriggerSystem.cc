#include "EventTriggerSystem.hh"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(gz::sim::EventTriggerSystem,
    gz::sim::System,
    gz::sim::EventTriggerSystem::ISystemConfigure,
    gz::sim::EventTriggerSystem::ISystemUpdate)

