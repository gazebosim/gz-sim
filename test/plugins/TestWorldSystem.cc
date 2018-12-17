#include "TestWorldSystem.hh"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(ignition::gazebo::TestWorldSystem,
    ignition::gazebo::System,
    ignition::gazebo::TestWorldSystem::ISystemConfigure)

