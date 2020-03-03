/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "CommandActor.hh"

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    command_actor::CommandActor,
    ignition::gazebo::System,
    command_actor::CommandActor::ISystemConfigure,
    command_actor::CommandActor::ISystemPreUpdate)
using namespace command_actor;

//////////////////////////////////////////////////
CommandActor::CommandActor()
{
}

//////////////////////////////////////////////////
CommandActor::~CommandActor()
{
}

//////////////////////////////////////////////////
void CommandActor::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &/*_ecm*/,
    ignition::gazebo::EventManager &/*_eventMgr*/)
{
  this->entity = _entity;
  ignmsg << "Command actor for entity [" << _entity << "]" << std::endl;

  auto sdfClone = _sdf->Clone();
  for (auto originElem = sdfClone->GetElement("origin");
       originElem != nullptr;
       originElem = originElem->GetNextElement())
  {
    auto pose = originElem->Get<ignition::math::Pose3d>("pose");
    auto time = originElem->Get<int>("time");

    this->origins[time] = pose;
    ignmsg << "Stored origin change at [" << time << "] seconds to pose ["
           << pose << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void CommandActor::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(
      _info.simTime).count();

  if (sec <= this->lastOriginChange)
    return;

  if (this->origins.find(sec) != this->origins.end())
  {
    auto poseComp = _ecm.Component<ignition::gazebo::components::Pose>(
        this->entity);
    *poseComp = ignition::gazebo::components::Pose(this->origins[sec]);

    // Updates to pose component can usually be missed, so we need to mark this
    // as an important one-time change.
    _ecm.SetChanged(this->entity, ignition::gazebo::components::Pose::typeId,
        ignition::gazebo::ComponentState::OneTimeChange);

    ignmsg << "Changing origin to [" << this->origins[sec] << "]" << std::endl;

    this->lastOriginChange = sec;
  }
}
