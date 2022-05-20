#include "SampleSystem.hh"

//! [registerSampleSystem2]
#include <gz/plugin/RegisterMore.hh>

IGNITION_ADD_PLUGIN(
    sample_system::SampleSystem2,
    gz::sim::System,
    sample_system::SampleSystem2::ISystemPreUpdate,
    sample_system::SampleSystem2::ISystemUpdate,
    sample_system::SampleSystem2::ISystemPostUpdate)
//! [registerSampleSystem2]

using namespace sample_system;

SampleSystem2::SampleSystem2()
{
}

SampleSystem2::~SampleSystem2()
{
}

void SampleSystem2::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "SampleSystem2::PreUpdate" << std::endl;
}

void SampleSystem2::Update(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "SampleSystem2::Update" << std::endl;
}

void SampleSystem2::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "SampleSystem2::PostUpdate" << std::endl;
}
