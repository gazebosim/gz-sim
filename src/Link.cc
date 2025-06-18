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

#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/Utility.hh>

#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/AxisAlignedBox.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/WindMode.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/Link.hh"

class gz::sim::LinkPrivate
{
  /// \brief Id of link entity.
  public: Entity id{kNullEntity};
};

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
Link::Link(sim::Entity _entity)
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
void Link::ResetEntity(sim::Entity _newEntity)
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
  return _ecm.ComponentData<components::Name>(this->dataPtr->id);
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
Entity Link::CollisionByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Collision());
}

//////////////////////////////////////////////////
Entity Link::SensorByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Sensor());
}

//////////////////////////////////////////////////
Entity Link::VisualByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Visual());
}

//////////////////////////////////////////////////
std::vector<Entity> Link::Collisions(const EntityComponentManager &_ecm) const
{
  return _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Collision());
}

//////////////////////////////////////////////////
std::vector<Entity> Link::Sensors(const EntityComponentManager &_ecm) const
{
  return _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Sensor());
}

//////////////////////////////////////////////////
std::vector<Entity> Link::Visuals(const EntityComponentManager &_ecm) const
{
  return _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Visual());
}

//////////////////////////////////////////////////
uint64_t Link::CollisionCount(const EntityComponentManager &_ecm) const
{
  return this->Collisions(_ecm).size();
}

//////////////////////////////////////////////////
uint64_t Link::SensorCount(const EntityComponentManager &_ecm) const
{
  return this->Sensors(_ecm).size();
}

//////////////////////////////////////////////////
uint64_t Link::VisualCount(const EntityComponentManager &_ecm) const
{
  return this->Visuals(_ecm).size();
}

//////////////////////////////////////////////////
bool Link::IsCanonical(const EntityComponentManager &_ecm) const
{
  auto comp = _ecm.Component<components::CanonicalLink>(this->dataPtr->id);
  return comp != nullptr;
}

//////////////////////////////////////////////////
bool Link::WindMode(const EntityComponentManager &_ecm) const
{
  auto comp = _ecm.Component<components::WindMode>(this->dataPtr->id);
  if (comp)
    return comp->Data();

  return false;
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Link::WorldPose(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::WorldPose>(this->dataPtr->id)
      .value_or(sim::worldPose(this->dataPtr->id, _ecm));
}

//////////////////////////////////////////////////
std::optional<math::Inertiald> Link::WorldInertial(
    const EntityComponentManager &_ecm) const
{
  auto inertial = _ecm.Component<components::Inertial>(this->dataPtr->id);
  auto worldPose = _ecm.ComponentData<components::WorldPose>(this->dataPtr->id)
                       .value_or(sim::worldPose(this->dataPtr->id, _ecm));

  if (!inertial)
    return std::nullopt;

  const math::Pose3d &comWorldPose =
      worldPose * inertial->Data().Pose();
  return std::make_optional(
    math::Inertiald(inertial->Data().MassMatrix(), comWorldPose));
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Link::WorldInertialPose(
    const EntityComponentManager &_ecm) const
{
  auto inertial = _ecm.Component<components::Inertial>(this->dataPtr->id);
  auto worldPose = _ecm.ComponentData<components::WorldPose>(this->dataPtr->id)
                       .value_or(sim::worldPose(this->dataPtr->id, _ecm));

  if (!inertial)
    return std::nullopt;

  return std::make_optional(worldPose * inertial->Data().Pose());
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Link::WorldLinearVelocity(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::WorldLinearVelocity>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Link::WorldLinearVelocity(
    const EntityComponentManager &_ecm,
    const math::Vector3d &_offset) const
{
  auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->id);
  auto worldPose = _ecm.ComponentData<components::WorldPose>(this->dataPtr->id)
                       .value_or(sim::worldPose(this->dataPtr->id, _ecm));
  auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->dataPtr->id);

  if (!worldLinVel || !worldAngVel)
    return std::nullopt;

  return std::make_optional(
      worldLinVel->Data() +
      worldAngVel->Data().Cross(worldPose.Rot().RotateVector(_offset)));
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Link::WorldAngularVelocity(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::WorldAngularVelocity>(
      this->dataPtr->id);
}

//////////////////////////////////////////////////
void Link::EnableVelocityChecks(EntityComponentManager &_ecm, bool _enable)
    const
{
  auto defaultWorldPose = math::Pose3d::Zero;
  if (_enable)
  {
    defaultWorldPose = sim::worldPose(this->dataPtr->id, _ecm);
  }

  enableComponent(_ecm, this->dataPtr->id,
    _enable, components::LinearVelocity());
  enableComponent(_ecm, this->dataPtr->id,
    _enable, components::AngularVelocity());
  enableComponent(_ecm, this->dataPtr->id,
    _enable, components::WorldPose(defaultWorldPose));

  auto defaultWorldLinVel = math::Vector3d::Zero;
  auto defaultWorldAngVel = math::Vector3d::Zero;
  if (_enable)
  {
    // The WorldPose component is guaranteed to exist at this point
    auto worldPose = this->WorldPose(_ecm).value();

    // Compute the default world linear and angular velocities
    defaultWorldLinVel = worldPose.Rot().RotateVector(
      _ecm.Component<components::LinearVelocity>(this->dataPtr->id)->Data());
    defaultWorldAngVel = worldPose.Rot().RotateVector(
      _ecm.Component<components::AngularVelocity>(this->dataPtr->id)->Data());
  }

  enableComponent(_ecm, this->dataPtr->id,
    _enable, components::WorldLinearVelocity(defaultWorldLinVel));
  enableComponent(_ecm, this->dataPtr->id,
    _enable, components::WorldAngularVelocity(defaultWorldAngVel));
}

//////////////////////////////////////////////////
void Link::SetLinearVelocity(EntityComponentManager &_ecm,
  const math::Vector3d &_vel) const
{
    auto vel =
      _ecm.Component<components::LinearVelocityCmd>(this->dataPtr->id);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(
          this->dataPtr->id,
          components::LinearVelocityCmd(_vel));
    }
    else
    {
      vel->Data() = _vel;
    }
}

//////////////////////////////////////////////////
void Link::SetAngularVelocity(EntityComponentManager &_ecm,
  const math::Vector3d &_vel) const
{
    auto vel =
      _ecm.Component<components::AngularVelocityCmd>(this->dataPtr->id);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(
          this->dataPtr->id,
          components::AngularVelocityCmd(_vel));
    }
    else
    {
      vel->Data() = _vel;
    }
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Link::WorldAngularAcceleration(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::WorldAngularAcceleration>(
      this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Link::WorldLinearAcceleration(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::WorldLinearAcceleration>(
      this->dataPtr->id);
}

//////////////////////////////////////////////////
void Link::EnableAccelerationChecks(EntityComponentManager &_ecm, bool _enable)
    const
{
  enableComponent<components::WorldAngularAcceleration>(_ecm, this->dataPtr->id,
      _enable);
  enableComponent<components::AngularAcceleration>(_ecm, this->dataPtr->id,
      _enable);
  enableComponent<components::WorldLinearAcceleration>(_ecm, this->dataPtr->id,
      _enable);
  enableComponent<components::LinearAcceleration>(_ecm, this->dataPtr->id,
      _enable);
}

//////////////////////////////////////////////////
std::optional<math::Matrix3d> Link::WorldInertiaMatrix(
    const EntityComponentManager &_ecm) const
{
  auto inertial = _ecm.Component<components::Inertial>(this->dataPtr->id);
  auto worldPose = _ecm.ComponentData<components::WorldPose>(this->dataPtr->id)
                       .value_or(sim::worldPose(this->dataPtr->id, _ecm));

  if (!inertial)
    return std::nullopt;

  const math::Pose3d &comWorldPose =
      worldPose * inertial->Data().Pose();
  return std::make_optional(
      math::Inertiald(inertial->Data().MassMatrix(), comWorldPose).Moi());
}

std::optional<math::Matrix6d> Link::WorldFluidAddedMassMatrix(
    const EntityComponentManager &_ecm) const
{
  auto inertial = _ecm.Component<components::Inertial>(this->dataPtr->id);
  auto worldPose = _ecm.Component<components::WorldPose>(this->dataPtr->id);

  if (!worldPose || !inertial)
    return std::nullopt;

  return inertial->Data().FluidAddedMass();
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
  auto worldPose = _ecm.ComponentData<components::WorldPose>(this->dataPtr->id)
                       .value_or(sim::worldPose(this->dataPtr->id, _ecm));

  // Can't apply force if the inertial's pose is not found
  if (!inertial)
    return;

  // We want the force to be applied at the center of mass, but
  // ExternalWorldWrenchCmd applies the force at the link origin so we need to
  // compute the resulting force and torque on the link origin.
  auto posComWorldCoord =
      worldPose.Rot().RotateVector(inertial->Data().Pose().Pos());

  math::Vector3d torque = posComWorldCoord.Cross(_force);

  this->AddWorldWrench(_ecm, _force, torque);
}

//////////////////////////////////////////////////
void Link::AddWorldForce(EntityComponentManager &_ecm,
                         const math::Vector3d &_force,
                         const math::Vector3d &_position) const
{
  auto inertial = _ecm.Component<components::Inertial>(this->dataPtr->id);
  auto worldPose = _ecm.ComponentData<components::WorldPose>(this->dataPtr->id)
                       .value_or(sim::worldPose(this->dataPtr->id, _ecm));

  // Can't apply force if the inertial's pose is not found
  if (!inertial)
    return;

  // We want the force to be applied at an offset from the center of mass, but
  // ExternalWorldWrenchCmd applies the force at the link origin so we need to
  // compute the resulting force and torque on the link origin.
  auto posComWorldCoord = worldPose.Rot().RotateVector(
    _position + inertial->Data().Pose().Pos());

  math::Vector3d torque = posComWorldCoord.Cross(_force);

  this->AddWorldWrench(_ecm, _force, torque);
}

//////////////////////////////////////////////////
void Link::AddWorldWrench(EntityComponentManager &_ecm,
                         const math::Vector3d &_force,
                         const math::Vector3d &_torque) const
{
  this->AddWorldWrench(_ecm, _force, _torque, math::Vector3d::Zero);
}

//////////////////////////////////////////////////
void Link::AddWorldWrench(EntityComponentManager &_ecm,
                          const math::Vector3d &_force,
                          const math::Vector3d &_torque,
                          const math::Vector3d &_offset) const
{
  math::Pose3d linkWorldPose;
  auto worldPoseComp = _ecm.Component<components::WorldPose>(this->dataPtr->id);
  if (worldPoseComp)
  {
    linkWorldPose = worldPoseComp->Data();
  }
  else
  {
    linkWorldPose = worldPose(this->dataPtr->id, _ecm);
  }

  // We want the force to be applied at an offset from the link origin, so we
  // must compute the resulting force and torque on the link origin.
  auto posComWorldCoord = linkWorldPose.Rot().RotateVector(_offset);
  math::Vector3d torqueWithOffset = _torque + posComWorldCoord.Cross(_force);

  auto linkWrenchComp =
    _ecm.Component<components::ExternalWorldWrenchCmd>(this->dataPtr->id);
  if (!linkWrenchComp)
  {
    components::ExternalWorldWrenchCmd wrench;
    msgs::Set(wrench.Data().mutable_force(), _force);
    msgs::Set(wrench.Data().mutable_torque(), torqueWithOffset);
    _ecm.CreateComponent(this->dataPtr->id, wrench);
  }
  else
  {
    msgs::Set(linkWrenchComp->Data().mutable_force(),
      msgs::Convert(linkWrenchComp->Data().force()) + _force);

    msgs::Set(linkWrenchComp->Data().mutable_torque(),
      msgs::Convert(linkWrenchComp->Data().torque()) + torqueWithOffset);
  }
}

//////////////////////////////////////////////////
void Link::EnableBoundingBoxChecks(
  EntityComponentManager & _ecm,
  bool _enable) const
{
  math::AxisAlignedBox linkAabb;
  if (_enable)
  {
    // Compute link's AABB from its collision shapes for proper initialization
    linkAabb = this->ComputeAxisAlignedBox(_ecm).value_or(
      math::AxisAlignedBox());
  }

  enableComponent(_ecm, this->dataPtr->id, _enable,
    components::AxisAlignedBox(linkAabb));
}

//////////////////////////////////////////////////
std::optional<math::AxisAlignedBox> Link::AxisAlignedBox(
  const EntityComponentManager & _ecm) const
{
  return _ecm.ComponentData<components::AxisAlignedBox>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<math::AxisAlignedBox> Link::WorldAxisAlignedBox(
  const EntityComponentManager & _ecm) const
{
  auto linkAabb = this->AxisAlignedBox(_ecm);

  if (!linkAabb.has_value())
  {
    return std::nullopt;
  }

  // Return the link AABB in the world frame
  return transformAxisAlignedBox(
    linkAabb.value(),
    this->WorldPose(_ecm).value()
  );
}

//////////////////////////////////////////////////
std::optional<math::AxisAlignedBox> Link::ComputeAxisAlignedBox(
  const EntityComponentManager & _ecm) const
{
  math::AxisAlignedBox linkAabb;
  auto collisions = this->Collisions(_ecm);

  if (collisions.empty())
  {
    return std::nullopt;
  }

  for (auto & entity : collisions)
  {
    auto collision = _ecm.ComponentData<components::CollisionElement>(entity);
    auto geom = collision.value().Geom();
    auto geomAabb = geom->AxisAlignedBox(&meshAxisAlignedBox);

    if (!geomAabb.has_value() || geomAabb == math::AxisAlignedBox())
    {
      gzwarn << "Failed to get bounding box for collision entity ["
             << entity << "]. It will be ignored in the computation "
             << "of the link bounding box." << std::endl;
      continue;
    }

    // Merge geometry AABB (expressed in link frame) into link AABB
    linkAabb += transformAxisAlignedBox(
      geomAabb.value(),
      _ecm.ComponentData<components::Pose>(entity).value()
    );
  }

  return linkAabb;
}
