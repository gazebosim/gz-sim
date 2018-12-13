#include "TestModelSystem.hh"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(ignition::gazebo::TestModelSystem,
    ignition::gazebo::System,
    ignition::gazebo::TestModelSystem::ISystemConfigure)

