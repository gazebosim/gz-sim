/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/ExternalWorldWrenchCmd.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "ignition/gazebo/Link.hh"

class ignition::gazebo::LinkPrivate
{
  /// \brief Id of link entity.
  public: Entity id{kNullEntity};
};

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
Link::Link(gazebo::Entity _entity)
  : dataPtr(std::make_unique<LinkPrivate>())
{
  this->dataPtr->id = _entity;
}

/////////////////////////////////////////////////
Link::Link(const Link &_link)
  : dataPtr(std::make_unique<LinkPrivate>(*_link.dataPtr))
{
}

/////////////////////////////////////////////////
Link::Link(Link &&_link) noexcept = default;

//////////////////////////////////////////////////
Link::~Link() = default;

/////////////////////////////////////////////////
Link &Link::operator=(const Link &_link)
{
  *this->dataPtr = (*_link.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Link &Link::operator=(Link &&_link) noexcept = default;

//////////////////////////////////////////////////
Entity Link::Entity() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
void Link::ResetEntity(gazebo::Entity _newEntity)
{
  this->dataPtr->id = _newEntity;
}

//////////////////////////////////////////////////
bool Link::Valid(const EntityComponentManager &_ecm) const
{
  return nullptr != _ecm.Component<components::Link>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::string> Link::Name(const EntityComponentManager &_ecm) const
{
  auto comp = _ecm.Component<components::Name>(this->dataPtr->id);
  if (!comp)
    return std::nullopt;

  return std::make_optional(comp->Data());
}

//////////////////////////////////////////////////
std::optional<Model> Link::ParentModel(const EntityComponentManager &_ecm) const
{
  auto parent = _ecm.Component<components::ParentEntity>(this->dataPtr->id);

  if (!parent)
    return std::nullopt;

  return std::optional<Model>(parent->Data());
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Link::WorldPose(
    const EntityComponentManager &_ecm) const
{
  auto worldPose = _ecm.Component<components::WorldPose>(this->dataPtr->id);
  if (!worldPose)
    return std::nullopt;

  return std::make_optional(worldPose->Data());
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Link::WorldInertialPose(
    const EntityComponentManager &_ecm) const
{
  auto inertial = _ecm.Component<components::Inertial>(this->dataPtr->id);
  auto worldPose = _ecm.Component<components::WorldPose>(this->dataPtr->id);

  if (!worldPose || !inertial)
    return std::nullopt;

  return std::make_optional(worldPose->Data() * inertial->Data().Pose());
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Link::WorldLinearVelocity(
    const EntityComponentManager &_ecm) const
{
  auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->id);

  if (!worldLinVel)
    return std::nullopt;

  return std::make_optional(worldLinVel->Data());
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Link::WorldLinearVelocity(
    const EntityComponentManager &_ecm,
    const math::Vector3d &_offset) const
{
  auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->id);
  auto worldPose =
      _ecm.Component<components::WorldPose>(this->dataPtr->id);
  auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->dataPtr->id);

  if (!worldLinVel || !worldPose || !worldAngVel)
    return std::nullopt;

  return std::make_optional(
      worldLinVel->Data() +
      worldAngVel->Data().Cross(worldPose->Data().Rot().RotateVector(_offset)));
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Link::WorldAngularVelocity(
    const EntityComponentManager &_ecm) const
{
  auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->dataPtr->id);

  if (!worldAngVel)
    return std::nullopt;

  return std::make_optional(worldAngVel->Data());
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Link::WorldLinearAcceleration(
    const EntityComponentManager &_ecm) const
{
  auto worldLinAccel =
      _ecm.Component<components::WorldLinearAcceleration>(this->dataPtr->id);

  if (!worldLinAccel)
    return std::nullopt;

  return std::make_optional(worldLinAccel->Data());
}

//////////////////////////////////////////////////
std::optional<math::Matrix3d> Link::WorldInertiaMatrix(
    const EntityComponentManager &_ecm) const
{
  auto inertial = _ecm.Component<components::Inertial>(this->dataPtr->id);
  auto worldPose = _ecm.Component<components::WorldPose>(this->dataPtr->id);

  if (!worldPose || !inertial)
    return std::nullopt;

  const math::Pose3d &comWorldPose =
      worldPose->Data() * inertial->Data().Pose();
  return std::make_optional(
      math::Inertiald(inertial->Data().MassMatrix(), comWorldPose).Moi());
}

//////////////////////////////////////////////////
std::optional<double> Link::WorldKineticEnergy(
    const EntityComponentManager &_ecm) const
{
  auto inertial = _ecm.Component<components::Inertial>(this->dataPtr->id);
  auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->dataPtr->id);

  if (!worldAngVel || !inertial)
    return std::nullopt;

  // get linear velocity at the center of mass using Inertial::Pose
  auto worldLinVel =
      this->WorldLinearVelocity(_ecm, inertial->Data().Pose().Pos());
  auto worldInertiaMatrix =
      this->WorldInertiaMatrix(_ecm);

  if (!worldLinVel || !worldInertiaMatrix)
    return std::nullopt;

  return std::make_optional(0.5 * (
      inertial->Data().MassMatrix().Mass() * worldLinVel->SquaredLength() +
      worldAngVel->Data().Dot(*worldInertiaMatrix * worldAngVel->Data())));
}

//////////////////////////////////////////////////
void Link::AddWorldForce(EntityComponentManager &_ecm,
                         const math::Vector3d &_force) const
{
  auto inertial = _ecm.Component<components::Inertial>(this->dataPtr->id);
  auto worldPose = _ecm.Component<components::WorldPose>(this->dataPtr->id);

  // Can't apply force if the inertial's pose is not found
  if (!inertial || !worldPose)
    return;

  // We want the force to be applied at the center of mass, but
  // ExternalWorldWrenchCmd applies the force at the link origin so we need to
  // compute the resulting force and torque on the link origin.
  auto posComWorldCoord =
      worldPose->Data().Rot().RotateVector(inertial->Data().Pose().Pos());

  math::Vector3d torque = posComWorldCoord.Cross(_force);

  this->AddWorldWrench(_ecm, _force, torque);
}

//////////////////////////////////////////////////
void Link::AddWorldWrench(EntityComponentManager &_ecm,
                         const math::Vector3d &_force,
                         const math::Vector3d &_torque) const
{
  auto linkWrenchComp =
      _ecm.Component<components::ExternalWorldWrenchCmd>(this->dataPtr->id);

  components::ExternalWorldWrenchCmd wrench;

  if (!linkWrenchComp)
  {
    msgs::Set(wrench.Data().mutable_force(), _force);
    msgs::Set(wrench.Data().mutable_torque(), _torque);
    _ecm.CreateComponent(this->dataPtr->id, wrench);
  }
  else
  {
    msgs::Set(linkWrenchComp->Data().mutable_force(),
              msgs::Convert(linkWrenchComp->Data().force()) + _force);

    msgs::Set(linkWrenchComp->Data().mutable_torque(),
              msgs::Convert(linkWrenchComp->Data().torque()) + _torque);
  }
}
