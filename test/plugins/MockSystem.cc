#include "MockSystem.hh"

#include <ignition/plugin/Register.hh>

using namespace ignition;
using namespace gazebo;

void MockSystem::PreUpdate(const gazebo::UpdateInfo & /*_info*/,
                gazebo::EntityComponentManager & /*_manager*/)
{
  ++this->preUpdateCallCount;
}

void MockSystem::Update(const gazebo::UpdateInfo & /*_info*/,
                gazebo::EntityComponentManager & /*_manager*/)
{
  ++this->updateCallCount;
}

void MockSystem::PostUpdate(const gazebo::UpdateInfo & /*_info*/,
              const gazebo::EntityComponentManager & /*_manager*/)
{
  ++this->postUpdateCallCount;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::MockSystem, ignition::gazebo::System,
    MockSystem::ISystemPreUpdate,
    MockSystem::ISystemUpdate,
    MockSystem::ISystemPostUpdate)

