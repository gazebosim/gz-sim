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

#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointEffortLimitsCmd.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionLimitsCmd.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointTransmittedWrench.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointVelocityLimitsCmd.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/ThreadPitch.hh"

#include "gz/sim/Joint.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;


class gz::sim::Joint::Implementation
{
  /// \brief Id of joint entity.
  public: sim::Entity id{kNullEntity};
};

//////////////////////////////////////////////////
Joint::Joint(sim::Entity _entity)
  : dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->id = _entity;
}

//////////////////////////////////////////////////
sim::Entity Joint::Entity() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
void Joint::ResetEntity(sim::Entity _newEntity)
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
std::optional<std::vector<sdf::JointAxis>> Joint::Axis(
    const EntityComponentManager &_ecm) const
{
  auto axis = _ecm.Component<components::JointAxis>(this->dataPtr->id);
  if (!axis)
    return std::nullopt;

  std::vector<sdf::JointAxis> axisVec{axis->Data()};

  auto axis2 = _ecm.Component<components::JointAxis2>(this->dataPtr->id);
  if (axis2)
    axisVec.push_back(axis2->Data());

  return std::optional<std::vector<sdf::JointAxis>>(axisVec);
}

//////////////////////////////////////////////////
std::optional<sdf::JointType> Joint::Type(
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
void Joint::SetVelocityLimits(EntityComponentManager &_ecm,
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
void Joint::SetEffortLimits(EntityComponentManager &_ecm,
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
void Joint::SetPositionLimits(EntityComponentManager &_ecm,
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

//////////////////////////////////////////////////
void Joint::ResetVelocity(EntityComponentManager &_ecm,
    const std::vector<double> &_velocities)
{
  auto jointVelocityReset =
    _ecm.Component<components::JointVelocityReset>(this->dataPtr->id);

  if (!jointVelocityReset)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::JointVelocityReset(_velocities));
  }
  else
  {
    jointVelocityReset->Data() = _velocities;
  }
}

//////////////////////////////////////////////////
void Joint::ResetPosition(EntityComponentManager &_ecm,
    const std::vector<double> &_positions)
{
  auto jointPositionReset =
    _ecm.Component<components::JointPositionReset>(this->dataPtr->id);

  if (!jointPositionReset)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::JointPositionReset(_positions));
  }
  else
  {
    jointPositionReset->Data() = _positions;
  }
}

//////////////////////////////////////////////////
void Joint::EnableVelocityCheck(EntityComponentManager &_ecm, bool _enable)
    const
{
  enableComponent<components::JointVelocity>(_ecm, this->dataPtr->id,
      _enable);
}

//////////////////////////////////////////////////
void Joint::EnablePositionCheck(EntityComponentManager &_ecm, bool _enable)
    const
{
  enableComponent<components::JointPosition>(_ecm, this->dataPtr->id,
      _enable);
}

//////////////////////////////////////////////////
void Joint::EnableTransmittedWrenchCheck(EntityComponentManager &_ecm,
    bool _enable) const
{
  enableComponent<components::JointTransmittedWrench>(_ecm, this->dataPtr->id,
      _enable);
}

//////////////////////////////////////////////////
std::optional<std::vector<double>> Joint::Velocity(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::JointVelocity>(
      this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::vector<double>> Joint::Position(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::JointPosition>(
      this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::vector<msgs::Wrench>> Joint::TransmittedWrench(
    const EntityComponentManager &_ecm) const
{
  // TransmittedWrench components contains one wrench msg value
  // instead of a vector like JointPosition and JointVelocity
  // components.
  // todo(anyone) change JointTransmittedWrench to store a vector
  // of wrench msgs?
  // We provide an API that returns a vector which is consistent
  // with Velocity and Position accessor functions
  auto comp = _ecm.Component<components::JointTransmittedWrench>(
      this->dataPtr->id);
  if (!comp)
    return std::nullopt;
  std::vector<msgs::Wrench> wrenchVec{comp->Data()};
  return wrenchVec;
}

//////////////////////////////////////////////////
std::optional<Model> Joint::ParentModel(const EntityComponentManager &_ecm)
    const
{
  auto parent = _ecm.Component<components::ParentEntity>(this->dataPtr->id);

  if (!parent)
    return std::nullopt;

  return std::optional<Model>(parent->Data());
}
