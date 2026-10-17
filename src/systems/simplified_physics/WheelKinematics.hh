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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_WHEEL_KINEMATICS_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_WHEEL_KINEMATICS_HH_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gz/math/Vector2.hh>

#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
  /// \brief Where a wheel touches the ground and which way it rolls. All
  /// vectors are in the model frame and in the ground plane.
  struct WheelGeometry
  {
    /// \brief Contact point, in meters from the model origin.
    math::Vector2d contact;

    /// \brief Unit vector along which the contact point travels when the
    /// joint turns in the positive direction. It is the joint axis crossed
    /// with the vertical.
    math::Vector2d rolling{1.0, 0.0};

    /// \brief Wheel radius, in meters.
    double radius{0.1};
  };

  /// \brief A planar velocity of the model origin, in the model frame.
  struct PlanarTwist
  {
    /// \brief Velocity along the model x axis, in m/s.
    double vx{0.0};

    /// \brief Velocity along the model y axis, in m/s.
    double vy{0.0};

    /// \brief Angular velocity about the model z axis, in rad/s.
    double wz{0.0};
  };

  /// \brief The rolling constraints of a set of wheels, in both directions.
  ///
  /// A wheel that rolls without slipping on a flat floor ties its angular
  /// rate w to the planar twist of the model through
  ///
  ///   w * r = d . (v + wz * (z x p))
  ///
  /// where p is the contact point, d the rolling direction and r the radius.
  /// The contact point must also not slide sideways:
  ///
  ///   n . (v + wz * (z x p)) = 0,  n = z x d.
  ///
  /// TwistFromRates solves these for the twist in the least squares sense.
  /// For two wheels on one axle the system is exact and gives the familiar
  /// v = r (wL + wR) / 2 and wz = r (wR - wL) / b. A model with more wheels
  /// than the twist has unknowns (a skid-steer vehicle) gets a compromise.
  /// The sideways rows count with a small weight, so that they only choose
  /// among motions that the rolling rows allow, for example the lateral
  /// velocity of a skid-steer vehicle, and do not cancel its turn.
  ///
  /// RatesFromTwist is the opposite direction: the rate that every wheel
  /// has to turn at for a twist, which is how the system moves the joint
  /// positions of wheels that follow the achieved motion.
  ///
  /// A mecanum wheel is not covered. Its rollers let the contact point slide
  /// along the roller axis, which this model forbids.
  class WheelDrive
  {
    /// \brief Most wheels one drive can hold. A wheel set is a 64 bit mask.
    public: static constexpr std::size_t kMaxWheels{64};

    /// \brief Weight of the sideways rows against the rolling rows.
    public: static constexpr double kSidewaysWeight{0.01};

    /// \brief Constructor.
    /// \param[in] _wheels The wheels. Wheels beyond kMaxWheels are dropped.
    public: explicit WheelDrive(std::vector<WheelGeometry> _wheels);

    /// \brief Number of wheels.
    /// \return The count.
    public: std::size_t Size() const;

    /// \brief A wheel of the drive.
    /// \param[in] _index Index below Size().
    /// \return Geometry of the wheel.
    public: const WheelGeometry &Wheel(std::size_t _index) const;

    /// \brief The largest distance between two contact points across the
    /// rolling direction of a wheel, a stand-in for the track width of the
    /// model. Wheels in a row along the rolling direction add nothing, so four
    /// wheels at (+-0.3, +-0.2) have a track of 0.4 m and not the 0.72 m of
    /// the diagonal.
    /// \return The distance in meters, zero for fewer than two wheels.
    public: double Track() const;

    /// \brief Planar twist of the model that makes the driven wheels roll at
    /// the given rates.
    ///
    /// The solution is cached for the last set of driven wheels, so a step
    /// costs one multiply-add per driven wheel while the set stays the same.
    /// A set with too few wheels to fix the twist (a single wheel, say) gets
    /// the solution of smallest norm.
    ///
    /// \param[in] _rates Angular rate of every wheel in rad/s, in the order
    /// of the constructor. Entries of wheels that are not driven are not
    /// read.
    /// \param[in] _driven Bit i is set if wheel i is driven.
    /// \return The twist at the model origin. Zero if no wheel is driven.
    public: PlanarTwist TwistFromRates(const double *_rates,
                std::uint64_t _driven);

    /// \brief Angular rate that a wheel needs for a twist.
    /// \param[in] _index Index below Size().
    /// \param[in] _twist Planar twist of the model origin.
    /// \return Rate in rad/s. Positive means the joint turns forward.
    public: double RateFromTwist(std::size_t _index,
                const PlanarTwist &_twist) const;

    /// \brief Solve for a set of driven wheels, replacing the cache.
    /// \param[in] _driven Bit i is set if wheel i is driven.
    private: void Factor(std::uint64_t _driven);

    /// \brief The wheels.
    private: std::vector<WheelGeometry> wheels;

    /// \brief Set of driven wheels that the coefficients are for.
    private: std::uint64_t cachedSet{0};

    /// \brief For every wheel, the contribution of its rolling speed w r to
    /// the twist, for the cached set. Zero for wheels outside the set.
    private: std::vector<std::array<double, 3>> coefficients;
  };

  /// \brief Advance a joint position by one step and keep it inside limits.
  ///
  /// \param[in] _position Position at the start, in rad or m.
  /// \param[in] _rate Commanded rate, in rad/s or m/s.
  /// \param[in] _dt Duration in seconds.
  /// \param[in] _lower Lower limit. A non-finite or huge value means none.
  /// \param[in] _upper Upper limit. A non-finite or huge value means none.
  /// \return The position at the end. A non-finite rate or duration, or a
  /// negative duration, leaves the position as it was (inside the limits).
  double AdvanceJoint(double _position, double _rate, double _dt,
      double _lower, double _upper);
}
}
}

#endif
