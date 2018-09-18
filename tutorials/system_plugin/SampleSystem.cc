#include "SampleSystem.hh"

#include <ignition/plugin/Register.hh>

using namespace sample_system;

SampleSystem::SampleSystem()
{
}

SampleSystem::~SampleSystem()
{
}

void SampleSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "SampleSystem::PostUpdate" << std::endl;
}

IGNITION_ADD_PLUGIN(
    sample_system::SampleSystem,
    ignition::gazebo::System,
    sample_system::SampleSystem::ISystemPostUpdate)


