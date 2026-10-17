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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_WHEEL_INFERENCE_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_WHEEL_INFERENCE_HH_

#include <string>
#include <vector>

#include <gz/math/Pose3.hh>

#include "ModelParams.hh"
#include "WheelKinematics.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
  /// \brief A joint that rolls a wheel along the floor.
  struct WheelJoint
  {
    /// \brief Joint entity.
    Entity joint{kNullEntity};

    /// \brief Where the wheel touches the floor and which way it rolls.
    WheelGeometry geometry;
  };

  /// \brief A one degree of freedom joint that is not a wheel.
  struct ArticulatedJoint
  {
    /// \brief Joint entity.
    Entity joint{kNullEntity};

    /// \brief Lower limit of the position, in rad or m. Huge values mean no
    /// limit.
    double lower{0.0};

    /// \brief Upper limit of the position.
    double upper{0.0};
  };

  /// \brief What each joint of a model does for the planar modes.
  struct JointRoles
  {
    /// \brief Joints that roll wheels on the floor.
    std::vector<WheelJoint> wheels;

    /// \brief Revolute, continuous and prismatic joints that are no wheels:
    /// lifts, turntables, steering joints, passive casters.
    std::vector<ArticulatedJoint> articulated;

    /// \brief Joints with a degree of freedom that the mode cannot follow:
    /// two-axis, screw and gearbox joints.
    std::vector<Entity> unsupported;
  };

  /// \brief Sort the joints of a model by what they do.
  ///
  /// Without `<wheel>` elements, a joint is a wheel if all of these hold:
  ///
  /// * it is revolute or continuous, with a horizontal axis (within 6
  ///   degrees);
  /// * its parent is the chassis: the canonical link or a link that is welded
  ///   to it by fixed joints;
  /// * its child link has a sphere collision, or a cylinder collision whose
  ///   axis is parallel to the joint axis (within 11 degrees);
  /// * the lowest point of that collision is within a few centimeters of the
  ///   lowest point of all box, sphere, cylinder, capsule and ellipsoid
  ///   collisions of the model, so the wheel touches the floor.
  ///
  /// A wheel that nothing commands (a passive wheel on a fixed axle) is a
  /// wheel too: it rolls, and its joint position follows. A caster that
  /// swivels is not one, because its rolling direction changes with the
  /// swivel, which the mode does not simulate. A ball joint is not a wheel.
  ///
  /// With `<wheel>` elements the named joints are the wheels and no other
  /// joint is. A named joint takes its radius from the element, or else from
  /// its collision. It needs no collision then: the joint origin stands in for
  /// the center. Every other revolute, continuous or prismatic joint is
  /// articulated.
  ///
  /// All geometry is in the model frame at the pose the model has now, which
  /// is assumed upright.
  ///
  /// \param[in] _ecm Constant reference to the ECM.
  /// \param[in] _model Top-level model.
  /// \param[in] _canonicalLink Canonical link of the model.
  /// \param[in] _modelPose World pose of the model.
  /// \param[in] _overrides The `<wheel>` elements, possibly empty.
  /// \param[in] _name Name of the model, for messages.
  /// \return The roles. Wheels are sorted by joint entity, so are the others.
  JointRoles ClassifyJoints(const EntityComponentManager &_ecm,
      const Entity &_model, const Entity &_canonicalLink,
      const math::Pose3d &_modelPose,
      const std::vector<WheelOverride> &_overrides, const std::string &_name);
}
}
}

#endif
