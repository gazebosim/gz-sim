#include "SampleSystem.hh"

//! [registerSampleSystem2]
#include <ignition/plugin/RegisterMore.hh>

IGNITION_ADD_PLUGIN(
    sample_system::SampleSystem2,
    ignition::gazebo::System,
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

void SampleSystem2::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "SampleSystem2::PreUpdate" << std::endl;
}

void SampleSystem2::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "SampleSystem2::Update" << std::endl;
}

void SampleSystem2::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "SampleSystem2::PostUpdate" << std::endl;
}
