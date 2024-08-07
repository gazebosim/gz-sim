#include "MockSystem.hh"

#include <gz/plugin/Register.hh>
#include <gz/plugin/RegisterMore.hh>

using namespace gz;
using namespace gz::sim;

IGNITION_ADD_PLUGIN(MockSystem, System,
    MockSystem::ISystemConfigure,
    MockSystem::ISystemPreUpdate,
    MockSystem::ISystemUpdate,
    MockSystem::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MockSystem, "gz::sim::MockSystem")
