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

#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(
    command_actor::CommandActor,
    gz::sim::System,
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
void CommandActor::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->entity = _entity;
  gzmsg << "Command actor for entity [" << _entity << "]" << std::endl;

  auto sdfClone = _sdf->Clone();

  // Origin demo
  for (auto originElem = sdfClone->GetElement("origin");
       originElem != nullptr;
       originElem = originElem->GetNextElement("origin"))
  {
    auto pose = originElem->Get<gz::math::Pose3d>("pose");
    auto time = originElem->Get<int>("time");

    this->origins[time] = pose;
    gzmsg << "Stored origin change at [" << time << "] seconds to pose ["
           << pose << "]" << std::endl;
  }

  // Trajectory pose demo
  for (auto trajPoseElem = sdfClone->GetElement("trajectory_pose");
       trajPoseElem != nullptr;
       trajPoseElem = trajPoseElem->GetNextElement("trajectory_pose"))
  {
    auto pose = trajPoseElem->Get<gz::math::Pose3d>("pose");
    auto time = trajPoseElem->Get<int>("time");

    this->trajPoses[time] = pose;
    gzmsg << "Stored trajectory pose change at [" << time
           << "] seconds to pose ["
           << pose << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void CommandActor::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(
      _info.simTime).count();

  // Update origins
  if (sec > this->lastOriginChange &&
      this->origins.find(sec) != this->origins.end())
  {
    auto poseComp = _ecm.Component<gz::sim::components::Pose>(
        this->entity);
    *poseComp = gz::sim::components::Pose(this->origins[sec]);

    // Updates to pose component can usually be missed, so we need to mark this
    // as an important one-time change.
    _ecm.SetChanged(this->entity, gz::sim::components::Pose::typeId,
        gz::sim::ComponentState::OneTimeChange);

    gzmsg << "Changing origin to [" << this->origins[sec] << "]" << std::endl;

    this->lastOriginChange = sec;
  }

  // Update trajectory poses
  if (sec > this->lastTrajPoseChange &&
      this->trajPoses.find(sec) != this->trajPoses.end())
  {
    auto trajPoseComp =
        _ecm.Component<gz::sim::components::TrajectoryPose>(
        this->entity);
    if (nullptr == trajPoseComp)
    {
      _ecm.CreateComponent(this->entity,
          gz::sim::components::TrajectoryPose(this->trajPoses[sec]));
    }
    else
    {
      *trajPoseComp =
          gz::sim::components::TrajectoryPose(this->trajPoses[sec]);
    }

    gzmsg << "Changing trajectory pose to [" << this->trajPoses[sec] << "]"
           << std::endl;

    this->lastTrajPoseChange = sec;
  }
}
