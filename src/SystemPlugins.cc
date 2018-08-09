
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/NullSystem.hh"
#include "ignition/gazebo/PhysicsSystem.hh"

#include "ignition/plugin/PluginMacros.hh"

IGN_PLUGIN_BEGIN_ADDING_PLUGINS
  IGN_PLUGIN_ADD_PLUGIN(ignition::gazebo::NullSystem, ignition::gazebo::System)
  IGN_PLUGIN_ADD_PLUGIN(ignition::gazebo::PhysicsSystem, ignition::gazebo::System)
IGN_PLUGIN_FINISH_ADDING_PLUGINS

