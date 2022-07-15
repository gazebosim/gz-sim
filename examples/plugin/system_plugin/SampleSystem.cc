#include "SampleSystem.hh"

//! [registerSampleSystem]
#include <gz/plugin/Register.hh>

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    sample_system::SampleSystem,
    gz::sim::System,
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

void SampleSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "SampleSystem::PostUpdate" << std::endl;
}
//! [implementSampleSystem]
