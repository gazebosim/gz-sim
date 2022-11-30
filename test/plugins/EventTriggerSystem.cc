#include "EventTriggerSystem.hh"

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(gz::sim::EventTriggerSystem,
    gz::sim::System,
    gz::sim::EventTriggerSystem::ISystemConfigure,
    gz::sim::EventTriggerSystem::ISystemUpdate)
