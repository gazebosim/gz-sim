
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/NullSystem.hh"
#include "ignition/gazebo/PhysicsSystem.hh"

#include "ignition/plugin/Register.hh"

IGNITION_ADD_PLUGIN(ignition::gazebo::NullSystem, ignition::gazebo::System)
IGNITION_ADD_PLUGIN(ignition::gazebo::PhysicsSystem, ignition::gazebo::System)

