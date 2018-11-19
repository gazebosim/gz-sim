#include "EventTriggerSystem.hh"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(ignition::gazebo::EventTriggerSystem,
    ignition::gazebo::System,
    ignition::gazebo::EventTriggerSystem::ISystemConfigure,
    ignition::gazebo::EventTriggerSystem::ISystemUpdate)

