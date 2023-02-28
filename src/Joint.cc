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

#include <ignition/msgs/Utility.hh>

#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointEffortLimitsCmd.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointPositionLimitsCmd.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/JointVelocityLimitsCmd.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
// #include "ignition/gazebo/Util.hh"

#include "ignition/gazebo/Joint.hh"

class ignition::gazebo::Joint::Implementation
{
  /// \brief Id of joint entity.
  public: gazebo::Entity id{kNullEntity};
};

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
Joint::Joint(gazebo::Entity _entity)
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->id = _entity;
}

//////////////////////////////////////////////////
gazebo::Entity Joint::Entity() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
void Joint::ResetEntity(gazebo::Entity _newEntity)
{
  this->dataPtr->id = _newEntity;
}

//////////////////////////////////////////////////
bool Joint::Valid(const EntityComponentManager &_ecm) const
{
  return nullptr != _ecm.Component<components::Joint>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::string> Joint::Name(const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::Name>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::string> Joint::ParentLinkName(
    const EntityComponentManager &_ecm) const
{
  auto parent = _ecm.Component<components::ParentLinkName>(this->dataPtr->id);

  if (!parent)
    return std::nullopt;

  return std::optional<std::string>(parent->Data());
}

//////////////////////////////////////////////////
std::optional<std::string> Joint::ChildLinkName(
    const EntityComponentManager &_ecm) const
{
  auto child = _ecm.Component<components::ChildLinkName>(this->dataPtr->id);

  if (!child)
    return std::nullopt;

  return std::optional<std::string>(child->Data());
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Joint::Pose(
    const EntityComponentManager &_ecm) const
{
  auto pose = _ecm.Component<components::Pose>(this->dataPtr->id);

  if (!pose)
    return std::nullopt;

  return std::optional<math::Pose3d>(pose->Data());
}

//////////////////////////////////////////////////
std::optional<double> Joint::ThreadPitch(
    const EntityComponentManager &_ecm) const
{
  auto threadPitch = _ecm.Component<components::ThreadPitch>(this->dataPtr->id);

  if (!threadPitch)
    return std::nullopt;

  return std::optional<double>(threadPitch->Data());
}

//////////////////////////////////////////////////
std::optional<sdf::JointAxis> Joint::JointAxis(
    const EntityComponentManager &_ecm, unsigned int _index) const
{
  if (_index == 0u)
  {
    auto axis = _ecm.Component<components::JointAxis>(this->dataPtr->id);
    if (!axis)
      return std::nullopt;
    return std::optional<sdf::JointAxis>(axis->Data());
  }
  else if (_index == 1u)
  {
    auto axis = _ecm.Component<components::JointAxis2>(this->dataPtr->id);
    if (!axis)
      return std::nullopt;
    return std::optional<sdf::JointAxis>(axis->Data());
  }
  else
  {
    ignwarn << "Axis index: [" << _index << "] is not supported" << std::endl;
    return std::nullopt;
  }
}

//////////////////////////////////////////////////
std::optional<sdf::JointType> Joint::JointType(
    const EntityComponentManager &_ecm) const
{
  auto jointType = _ecm.Component<components::JointType>(this->dataPtr->id);

  if (!jointType)
    return std::nullopt;

  return std::optional<sdf::JointType>(jointType->Data());
}

//////////////////////////////////////////////////
Entity Joint::SensorByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Sensor());
}

//////////////////////////////////////////////////
std::vector<Entity> Joint::Sensors(const EntityComponentManager &_ecm) const
{
  return _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Sensor());
}

//////////////////////////////////////////////////
uint64_t Joint::SensorCount(const EntityComponentManager &_ecm) const
{
  return this->Sensors(_ecm).size();
}

//////////////////////////////////////////////////
void Joint::SetVelocity(EntityComponentManager &_ecm,
    const std::vector<double> &_velocities)
{
  auto jointVelocityCmd =
    _ecm.Component<components::JointVelocityCmd>(this->dataPtr->id);

  if (!jointVelocityCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::JointVelocityCmd(_velocities));
  }
  else
  {
    jointVelocityCmd->Data() = _velocities;
  }
}

//////////////////////////////////////////////////
void Joint::SetForce(EntityComponentManager &_ecm,
    const std::vector<double> &_forces)
{
  auto jointForceCmd =
    _ecm.Component<components::JointForceCmd>(this->dataPtr->id);

  if (!jointForceCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::JointForceCmd(_forces));
  }
  else
  {
    jointForceCmd->Data() = _forces;
  }
}

//////////////////////////////////////////////////
void Joint::SetVelocityLimit(EntityComponentManager &_ecm,
    const std::vector<math::Vector2d> &_limits)
{
  auto jointVelocityLimitsCmd =
    _ecm.Component<components::JointVelocityLimitsCmd>(this->dataPtr->id);

  if (!jointVelocityLimitsCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::JointVelocityLimitsCmd(_limits));
  }
  else
  {
    jointVelocityLimitsCmd->Data() = _limits;
  }
}

//////////////////////////////////////////////////
void Joint::SetEffortLimit(EntityComponentManager &_ecm,
    const std::vector<math::Vector2d> &_limits)
{
  auto jointEffortLimitsCmd =
    _ecm.Component<components::JointEffortLimitsCmd>(this->dataPtr->id);

  if (!jointEffortLimitsCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::JointEffortLimitsCmd(_limits));
  }
  else
  {
    jointEffortLimitsCmd->Data() = _limits;
  }
}

//////////////////////////////////////////////////
void Joint::SetPositionLimit(EntityComponentManager &_ecm,
    const std::vector<math::Vector2d> &_limits)
{
  auto jointPosLimitsCmd =
    _ecm.Component<components::JointPositionLimitsCmd>(this->dataPtr->id);

  if (!jointPosLimitsCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::JointPositionLimitsCmd(_limits));
  }
  else
  {
    jointPosLimitsCmd->Data() = _limits;
  }
}
