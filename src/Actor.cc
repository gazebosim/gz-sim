/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"

#include "gz/sim/Actor.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;

class gz::sim::Actor::Implementation
{
  /// \brief Id of actor entity.
  public: sim::Entity id{kNullEntity};
};

//////////////////////////////////////////////////
Actor::Actor(sim::Entity _entity)
  : dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->id = _entity;
}

//////////////////////////////////////////////////
sim::Entity Actor::Entity() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
void Actor::ResetEntity(sim::Entity _newEntity)
{
  this->dataPtr->id = _newEntity;
}

//////////////////////////////////////////////////
bool Actor::Valid(const EntityComponentManager &_ecm) const
{
  return nullptr != _ecm.Component<components::Actor>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::string> Actor::Name(const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::Name>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Actor::Pose(
    const EntityComponentManager &_ecm) const
{
  auto pose = _ecm.Component<components::Pose>(this->dataPtr->id);

  if (!pose)
    return std::nullopt;

  return std::optional<math::Pose3d>(pose->Data());
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Actor::WorldPose(
    const EntityComponentManager &_ecm) const
{
  auto pose = _ecm.Component<components::WorldPose>(this->dataPtr->id);

  if (!pose)
    return std::nullopt;

  return std::optional<math::Pose3d>(pose->Data());
}

//////////////////////////////////////////////////
void Actor::SetTrajectoryPose(EntityComponentManager &_ecm,
    const math::Pose3d &_pose)
{
  auto pose =
    _ecm.Component<components::TrajectoryPose>(this->dataPtr->id);

  if (!pose)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::TrajectoryPose(_pose));
  }
  else
  {
    pose->Data() = _pose;
  }
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Actor::TrajectoryPose(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::TrajectoryPose>(
      this->dataPtr->id);
}

//////////////////////////////////////////////////
void Actor::SetAnimationName(EntityComponentManager &_ecm,
    const std::string &_name)
{
  auto animName =
    _ecm.Component<components::AnimationName>(this->dataPtr->id);

  if (!animName)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::AnimationName(_name));
  }
  else
  {
    animName->Data() = _name;
  }
}

//////////////////////////////////////////////////
void Actor::SetAnimationTime(EntityComponentManager &_ecm,
    const std::chrono::steady_clock::duration &_time)
{
  auto animTime =
    _ecm.Component<components::AnimationTime>(this->dataPtr->id);

  if (!animTime)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::AnimationTime(_time));
  }
  else
  {
    animTime->Data() = _time;
  }
}

//////////////////////////////////////////////////
std::optional<std::string> Actor::AnimationName(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::AnimationName>(
      this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::chrono::steady_clock::duration> Actor::AnimationTime(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::AnimationTime>(
      this->dataPtr->id);
}
