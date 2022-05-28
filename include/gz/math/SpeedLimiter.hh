/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GZ_MATH_SYSTEMS_SPEEDLIMITER_HH_
#define GZ_MATH_SYSTEMS_SPEEDLIMITER_HH_

#include <chrono>
#include <memory>
#include <gz/math/config.hh>
#include "gz/math/Helpers.hh"

namespace gz
{
namespace math
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_MATH_VERSION_NAMESPACE {
  // Forward declaration.
  class SpeedLimiterPrivate;

  /// \brief Class to limit velocity, acceleration and jerk.
  class GZ_MATH_VISIBLE SpeedLimiter
  {
    /// \brief Constructor.
    /// There are no limits by default.
    public: SpeedLimiter();

    /// \brief Destructor.
    public: ~SpeedLimiter();

    /// \brief Set minimum velocity limit in m/s, usually <= 0.
    /// \param[in] _lim Minimum velocity.
    public: void SetMinVelocity(double _lim);

    /// \brief Get minimum velocity limit, defaults to negative infinity.
    /// \return Minimum velocity.
    public: double MinVelocity() const;

    /// \brief Set maximum velocity limit in m/s, usually >= 0.
    /// \param[in] _lim Maximum velocity.
    public: void SetMaxVelocity(double _lim);

    /// \brief Get maximum velocity limit, defaults to positive infinity.
    /// \return Maximum velocity.
    public: double MaxVelocity() const;

    /// \brief Set minimum acceleration limit in m/s^2, usually <= 0.
    /// \param[in] _lim Minimum acceleration.
    public: void SetMinAcceleration(double _lim);

    /// \brief Get minimum acceleration limit, defaults to negative infinity.
    /// \return Minimum acceleration.
    public: double MinAcceleration() const;

    /// \brief Set maximum acceleration limit in m/s^2, usually >= 0.
    /// \param[in] _lim Maximum acceleration.
    public: void SetMaxAcceleration(double _lim);

    /// \brief Get maximum acceleration limit, defaults to positive infinity.
    /// \return Maximum acceleration.
    public: double MaxAcceleration() const;

    /// \brief Set minimum jerk limit in m/s^3, usually <= 0.
    /// \param[in] _lim Minimum jerk.
    public: void SetMinJerk(double _lim);

    /// \brief Get minimum jerk limit, defaults to negative infinity.
    /// \return Minimum jerk.
    public: double MinJerk() const;

    /// \brief Set maximum jerk limit in m/s^3, usually >= 0.
    /// \param[in] _lim Maximum jerk.
    public: void SetMaxJerk(double _lim);

    /// \brief Get maximum jerk limit, defaults to positive infinity.
    /// \return Maximum jerk.
    public: double MaxJerk() const;

    /// \brief Limit velocity, acceleration and jerk.
    /// \param [in, out] _vel Velocity to limit [m/s].
    /// \param [in] _prevVel Previous velocity to _vel [m/s].
    /// \param [in] _prevPrevVel Previous velocity to _prevVel [m/s].
    /// \param [in] _dt Time step.
    /// \return Limiting difference, which is (out _vel - in _vel).
    public: double Limit(double &_vel,
                         double _prevVel,
                         double _prevPrevVel,
                         std::chrono::steady_clock::duration _dt) const;

    /// \brief Limit the velocity.
    /// \param [in, out] _vel Velocity to limit [m/s].
    /// \return Limiting difference, which is (out _vel - in _vel).
    public: double LimitVelocity(double &_vel) const;

    /// \brief Limit the acceleration using a first-order backward difference
    /// method.
    /// \param [in, out] _vel  Velocity [m/s].
    /// \param [in] _prevVel Previous velocity [m/s].
    /// \param [in] _dt Time step.
    /// \return Limiting difference, which is (out _vel - in _vel).
    public: double LimitAcceleration(
        double &_vel,
        double _prevVel,
        std::chrono::steady_clock::duration _dt) const;

    /// \brief Limit the jerk using a second-order backward difference method.
    /// \param [in, out] _vel Velocity to limit [m/s].
    /// \param [in] _prevVel Previous velocity to v  [m/s].
    /// \param [in] _prevPrevVel Previous velocity to prevVel [m/s].
    /// \param [in] _dt Time step.
    /// \return Limiting difference, which is (out _vel - in _vel).
    /// \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control.
    public: double LimitJerk(
        double &_vel,
        double _prevVel,
        double _prevPrevVel,
        std::chrono::steady_clock::duration _dt) const;

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
    /// \brief Private data pointer.
    private: std::unique_ptr<SpeedLimiterPrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
  };
  }
}
}

#endif
