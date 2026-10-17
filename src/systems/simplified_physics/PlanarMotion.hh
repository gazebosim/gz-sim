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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_PLANAR_MOTION_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_PLANAR_MOTION_HH_

#include <cmath>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "WheelKinematics.hh"
#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
  /// \brief Displacement of a planar body over one step.
  struct PlanarDelta
  {
    /// \brief Displacement of the body origin along the world x axis, in
    /// meters.
    double x{0.0};

    /// \brief Displacement of the body origin along the world y axis, in
    /// meters.
    double y{0.0};

    /// \brief Change of heading about the world z axis, in radians.
    double yaw{0.0};

    /// \brief True if the body does not move.
    /// \return True if all three components are exactly zero.
    bool IsZero() const
    {
      return std::fpclassify(this->x) == FP_ZERO &&
          std::fpclassify(this->y) == FP_ZERO &&
          std::fpclassify(this->yaw) == FP_ZERO;
    }
  };

  /// \brief Integrate a planar twist that is constant in the body frame.
  ///
  /// A body that keeps the same linear and angular velocity in its own frame
  /// drives along a circular arc. This function returns the exact
  /// displacement along that arc, so the result for a given command and
  /// duration does not depend on how the duration is split into steps.
  ///
  /// With theta = _wz * _dt, the displacement in the heading frame is
  ///
  ///   dx = _dt * (_vx * sinc(theta) - _vy * (1 - cos(theta)) / theta)
  ///   dy = _dt * (_vy * sinc(theta) + _vx * (1 - cos(theta)) / theta)
  ///
  /// and a rotation by _yaw takes it to the world frame. Both factors have a
  /// finite limit at theta = 0. A series is used there, so the function never
  /// divides by a small number.
  ///
  /// A non-positive or non-finite _dt, or a non-finite twist, gives a zero
  /// displacement.
  ///
  /// \param[in] _yaw Heading of the body at the start, in radians.
  /// \param[in] _vx Velocity along the heading axis, in m/s.
  /// \param[in] _vy Velocity along the axis to the left of the heading, in
  /// m/s.
  /// \param[in] _wz Angular velocity about the world z axis, in rad/s.
  /// \param[in] _dt Duration, in seconds.
  /// \return Displacement of the body origin and the change of heading.
  PlanarDelta IntegrateArc(double _yaw, double _vx, double _vy, double _wz,
      double _dt);

  /// \brief The body-frame twist that IntegrateArc turns into a displacement.
  ///
  /// This is the inverse of IntegrateArc for one step. A motion that
  /// collisions cut is a displacement and a turn that no command asked for,
  /// and the wheel joints and the velocities that readers see must follow it.
  /// The inverse of the 2x2 matrix [s -c; c s] of IntegrateArc is
  /// [s c; -c s] / (s^2 + c^2), with the same series for small angles.
  ///
  /// A non-positive or non-finite _dt, or a non-finite displacement, gives a
  /// zero twist.
  ///
  /// \param[in] _yaw Heading of the body at the start, in radians.
  /// \param[in] _delta Displacement of the body origin and change of heading.
  /// \param[in] _dt Duration, in seconds.
  /// \return Velocity of the body origin in the body frame, and the angular
  /// velocity, that IntegrateArc would integrate to _delta.
  PlanarTwist TwistFromDelta(double _yaw, const PlanarDelta &_delta,
      double _dt);

  /// \brief Pose, velocity and acceleration of a frame, all in the world
  /// frame.
  struct RigidFrame
  {
    /// \brief Pose of the frame.
    math::Pose3d pose;

    /// \brief Linear velocity of the frame origin.
    math::Vector3d linearVelocity;

    /// \brief Angular velocity of the frame.
    math::Vector3d angularVelocity;

    /// \brief Linear acceleration of the frame origin.
    math::Vector3d linearAcceleration;

    /// \brief Angular acceleration of the frame.
    math::Vector3d angularAcceleration;
  };

  /// \brief Frame data of a point that is fixed to a planar body.
  ///
  /// The body turns at a constant rate about the world z axis and its origin
  /// keeps a constant velocity in the body frame, as IntegrateArc assumes. The
  /// angular acceleration is then zero and a point accelerates toward the
  /// center of the turn: a = w x v. A change of the command between two
  /// steps is not differentiated, so it does not show up as an acceleration.
  ///
  /// The function also gives the frame data of a sensor or a collision on a
  /// link, when the origin is the link's frame.
  ///
  /// \param[in] _origin World pose of the body frame.
  /// \param[in] _linearVelocity World linear velocity of the body origin.
  /// \param[in] _wz Angular velocity of the body about the world z axis.
  /// \param[in] _offset Pose of the point in the body frame.
  /// \return Frame data of the point.
  RigidFrame PlanarFrameAt(const math::Pose3d &_origin,
      const math::Vector3d &_linearVelocity, double _wz,
      const math::Pose3d &_offset);
}
}
}

#endif
