/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "WheelInference.hh"

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Sphere.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
namespace
{
/// \brief Largest vertical part of a unit axis vector for a horizontal axis:
/// sin(6 degrees).
constexpr double kMaxAxisTilt{0.105};

/// \brief Smallest |cos| between a cylinder's axis and the joint axis: cos(11
/// degrees).
constexpr double kMinCylinderAlignment{0.98};

/// \brief How far above the lowest collision point the bottom of a wheel may
/// be, as a fraction of its radius, and at most this many meters.
constexpr double kGroundToleranceFraction{0.25};
constexpr double kGroundToleranceMax{0.02};

/// \brief A collision shape in the model frame.
struct Shape
{
  /// \brief Pose in the model frame.
  math::Pose3d pose;

  /// \brief The sdf geometry.
  sdf::Geometry geometry;
};

/// \brief Lowest point of a collision, if the shape has a simple form.
/// \param[in] _shape The shape.
/// \return Height of the lowest point in the model frame, or nullopt for
/// shapes this file does not bound (meshes, planes, heightmaps).
std::optional<double> LowestPoint(const Shape &_shape)
{
  const auto &g = _shape.geometry;
  const math::Matrix3d r(_shape.pose.Rot());
  // The model z axis in the collision frame is the third row of r.
  const double zx = r(2, 0);
  const double zy = r(2, 1);
  const double zz = r(2, 2);
  const double z = _shape.pose.Pos().Z();

  switch (g.Type())
  {
    case sdf::GeometryType::BOX:
    {
      const auto half = g.BoxShape()->Size() * 0.5;
      return z - (half.X() * std::abs(zx) + half.Y() * std::abs(zy) +
          half.Z() * std::abs(zz));
    }
    case sdf::GeometryType::SPHERE:
      return z - g.SphereShape()->Radius();
    case sdf::GeometryType::CYLINDER:
    {
      const double radius = g.CylinderShape()->Radius();
      const double half = 0.5 * g.CylinderShape()->Length();
      return z - (radius * std::sqrt(std::max(0.0, 1.0 - zz * zz)) +
          half * std::abs(zz));
    }
    case sdf::GeometryType::CAPSULE:
    {
      const double radius = g.CapsuleShape()->Radius();
      const double half = 0.5 * g.CapsuleShape()->Length();
      return z - (radius + half * std::abs(zz));
    }
    case sdf::GeometryType::ELLIPSOID:
    {
      const auto radii = g.EllipsoidShape()->Radii();
      return z - std::sqrt(std::pow(radii.X() * zx, 2) +
          std::pow(radii.Y() * zy, 2) + std::pow(radii.Z() * zz, 2));
    }
    default:
      return std::nullopt;
  }
}

/// \brief Find a link of a joint's parent model by name.
/// \param[in] _ecm Constant reference to the ECM.
/// \param[in] _joint Joint entity.
/// \param[in] _name Link name as the joint stores it.
/// \return The link, or kNullEntity if the name is scoped or unknown.
Entity FindLink(const EntityComponentManager &_ecm, const Entity &_joint,
    const std::string &_name)
{
  if (_name.find("::") != std::string::npos)
    return kNullEntity;

  const auto *parent = _ecm.Component<components::ParentEntity>(_joint);
  if (!parent)
    return kNullEntity;

  return _ecm.EntityByComponents(components::Link(),
      components::ParentEntity(parent->Data()), components::Name(_name));
}

/// \brief Collision shapes of a link in the model frame.
std::vector<Shape> LinkShapes(const EntityComponentManager &_ecm,
    const Entity &_link, const math::Pose3d &_modelPose)
{
  std::vector<Shape> shapes;
  for (const auto &collision : Link(_link).Collisions(_ecm))
  {
    const auto *geometry = _ecm.Component<components::Geometry>(collision);
    if (!geometry)
      continue;
    shapes.push_back({_modelPose.Inverse() * worldPose(collision, _ecm),
        geometry->Data()});
  }
  return shapes;
}
}

//////////////////////////////////////////////////
JointRoles ClassifyJoints(const EntityComponentManager &_ecm,
    const Entity &_model, const Entity &_canonicalLink,
    const math::Pose3d &_modelPose,
    const std::vector<WheelOverride> &_overrides, const std::string &_name)
{
  JointRoles roles;

  std::vector<Entity> joints;
  std::vector<Entity> collisions;
  for (const auto &entity : _ecm.Descendants(_model))
  {
    if (_ecm.Component<components::Joint>(entity))
      joints.push_back(entity);
    else if (_ecm.Component<components::Collision>(entity))
      collisions.push_back(entity);
  }
  std::sort(joints.begin(), joints.end());

  // The chassis is the canonical link and every link that is welded to it,
  // directly or through other welded links. A model often puts a massless
  // footprint link first and hangs the real base on it with a fixed joint.
  std::set<Entity> chassis{_canonicalLink};
  std::vector<std::pair<Entity, Entity>> welds;
  for (const auto &joint : joints)
  {
    const auto *type = _ecm.Component<components::JointType>(joint);
    const auto *parentName = _ecm.Component<components::ParentLinkName>(joint);
    const auto *childName = _ecm.Component<components::ChildLinkName>(joint);
    if (!type || type->Data() != sdf::JointType::FIXED || !parentName ||
        !childName)
    {
      continue;
    }
    const Entity parent = FindLink(_ecm, joint, parentName->Data());
    const Entity child = FindLink(_ecm, joint, childName->Data());
    if (parent != kNullEntity && child != kNullEntity)
      welds.emplace_back(parent, child);
  }
  for (bool grew = true; grew;)
  {
    grew = false;
    for (const auto &[a, b] : welds)
    {
      if (chassis.count(a) != chassis.count(b))
      {
        chassis.insert({a, b});
        grew = true;
      }
    }
  }

  // Lowest point of the model, for the "touches the floor" test.
  std::optional<double> lowest;
  for (const auto &collision : collisions)
  {
    const auto *geometry = _ecm.Component<components::Geometry>(collision);
    if (!geometry)
      continue;
    const auto bottom = LowestPoint({
        _modelPose.Inverse() * worldPose(collision, _ecm), geometry->Data()});
    if (bottom && (!lowest || *bottom < *lowest))
      lowest = bottom;
  }

  std::vector<bool> overrideUsed(_overrides.size(), false);

  for (const auto &joint : joints)
  {
    const auto *type = _ecm.Component<components::JointType>(joint);
    const auto *axisComp = _ecm.Component<components::JointAxis>(joint);
    if (!type)
      continue;

    switch (type->Data())
    {
      case sdf::JointType::REVOLUTE:
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::PRISMATIC:
        break;
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::UNIVERSAL:
      case sdf::JointType::SCREW:
      case sdf::JointType::GEARBOX:
        roles.unsupported.push_back(joint);
        continue;
      default:
        // Ball and fixed joints have no position to follow.
        continue;
    }
    if (!axisComp)
      continue;

    const auto jointName =
        _ecm.ComponentData<components::Name>(joint).value_or("");
    ArticulatedJoint articulated;
    articulated.joint = joint;
    articulated.lower = axisComp->Data().Lower();
    articulated.upper = axisComp->Data().Upper();

    if (type->Data() == sdf::JointType::PRISMATIC)
    {
      roles.articulated.push_back(articulated);
      continue;
    }

    const auto *parentName =
        _ecm.Component<components::ParentLinkName>(joint);
    const auto *childName = _ecm.Component<components::ChildLinkName>(joint);
    const Entity parentLink = parentName ?
        FindLink(_ecm, joint, parentName->Data()) : kNullEntity;
    const Entity childLink = childName ?
        FindLink(_ecm, joint, childName->Data()) : kNullEntity;
    if (childLink == kNullEntity)
    {
      roles.articulated.push_back(articulated);
      continue;
    }

    // Joint axis in the model frame. The stored axis is in the joint frame,
    // and the stored joint pose is relative to the child link.
    const auto jointPose = _modelPose.Inverse() * worldPose(childLink, _ecm) *
        _ecm.ComponentData<components::Pose>(joint).value_or(math::Pose3d());
    math::Vector3d axis = jointPose.Rot().RotateVector(axisComp->Data().Xyz());
    if (axis.Length() < 1e-9)
    {
      roles.articulated.push_back(articulated);
      continue;
    }
    axis.Normalize();

    // Which wheel element, if any, names this joint.
    const WheelOverride *named{nullptr};
    for (std::size_t i = 0; i < _overrides.size(); ++i)
    {
      if (_overrides[i].joint == jointName)
      {
        named = &_overrides[i];
        overrideUsed[i] = true;
        break;
      }
    }

    const bool explicitWheels = !_overrides.empty();
    if (explicitWheels && !named)
    {
      roles.articulated.push_back(articulated);
      continue;
    }

    if (std::abs(axis.Z()) > kMaxAxisTilt)
    {
      if (named)
      {
        gzwarn << "Model [" << _name << "] names joint [" << jointName
               << "] as a wheel, but its axis is not horizontal. The joint is "
               << "not a wheel." << std::endl;
      }
      roles.articulated.push_back(articulated);
      continue;
    }

    // The parent must be the chassis, and the child must roll.
    if (!named && chassis.count(parentLink) == 0)
    {
      roles.articulated.push_back(articulated);
      continue;
    }

    // The wheel shape: a sphere, or a cylinder along the axis.
    std::optional<Shape> wheelShape;
    for (const auto &shape : LinkShapes(_ecm, childLink, _modelPose))
    {
      if (shape.geometry.Type() == sdf::GeometryType::SPHERE)
      {
        wheelShape = shape;
        break;
      }
      if (shape.geometry.Type() == sdf::GeometryType::CYLINDER)
      {
        const auto cylinderAxis =
            shape.pose.Rot().RotateVector(math::Vector3d::UnitZ);
        if (std::abs(cylinderAxis.Dot(axis)) >= kMinCylinderAlignment)
        {
          wheelShape = shape;
          break;
        }
      }
    }

    double radius{0.0};
    math::Vector3d center = jointPose.Pos();
    if (wheelShape)
    {
      center = wheelShape->pose.Pos();
      radius = wheelShape->geometry.Type() == sdf::GeometryType::SPHERE ?
          wheelShape->geometry.SphereShape()->Radius() :
          wheelShape->geometry.CylinderShape()->Radius();
    }
    if (named && named->radius)
      radius = *named->radius;

    if (radius <= 0.0 || (!wheelShape && !named))
    {
      if (named)
      {
        gzwarn << "Model [" << _name << "] names joint [" << jointName
               << "] as a wheel, but gives no radius and the child link has "
               << "no sphere or cylinder collision. The joint is not a wheel."
               << std::endl;
      }
      roles.articulated.push_back(articulated);
      continue;
    }

    if (!named && lowest)
    {
      const double tolerance =
          std::min(kGroundToleranceMax, kGroundToleranceFraction * radius);
      if (center.Z() - radius > *lowest + tolerance)
      {
        roles.articulated.push_back(articulated);
        continue;
      }
    }

    WheelJoint wheel;
    wheel.joint = joint;
    wheel.geometry.contact.Set(center.X(), center.Y());
    // Rolling direction: the axis crossed with the vertical.
    const math::Vector2d horizontal(axis.X(), axis.Y());
    const double length = horizontal.Length();
    wheel.geometry.rolling.Set(horizontal.Y() / length,
        -horizontal.X() / length);
    wheel.geometry.radius = radius;
    roles.wheels.push_back(wheel);
  }

  for (std::size_t i = 0; i < _overrides.size(); ++i)
  {
    if (!overrideUsed[i])
    {
      gzwarn << "Model [" << _name << "] names wheel joint ["
             << _overrides[i].joint << "], which is not a revolute or "
             << "continuous joint of the model. It is ignored." << std::endl;
    }
  }
  return roles;
}
}
}
}
