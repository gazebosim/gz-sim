#include "SampleSystem.hh"

//! [registerSampleSystem]
#include <ignition/plugin/Register.hh>

// Include a line in your source file for each interface implemented.
IGNITION_ADD_PLUGIN(
    sample_system::SampleSystem,
    ignition::gazebo::System,
    sample_system::SampleSystem::ISystemPostUpdate)
//! [registerSampleSystem]
//! [implementSampleSystem]
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
//! [implementSampleSystem]
