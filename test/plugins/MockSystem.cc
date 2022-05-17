#include "MockSystem.hh"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(ignition::gazebo::MockSystem, ignition::gazebo::System,
    ignition::gazebo::MockSystem::ISystemConfigure,
    ignition::gazebo::MockSystem::ISystemReset,
    ignition::gazebo::MockSystem::ISystemPreUpdate,
    ignition::gazebo::MockSystem::ISystemUpdate,
    ignition::gazebo::MockSystem::ISystemPostUpdate)

