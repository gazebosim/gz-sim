/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Enrique Fernández
 * Modified: Carlos Agüero
 */

#ifndef IGNITION_GAZEBO_SYSTEMS_SPEEDLIMITER_HH_
#define IGNITION_GAZEBO_SYSTEMS_SPEEDLIMITER_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration.
  class SpeedLimiterPrivate;

  /// \brief Class to limit velocity, acceleration and jerk.
  /// \ref https://github.com/ros-controls/ros_controllers/tree/melodic-devel/diff_drive_controller
  class IGNITION_GAZEBO_VISIBLE SpeedLimiter
  {
    /// \brief Constructor.
    /// \param [in] _hasVelocityLimits     if true, applies velocity limits.
    /// \param [in] _hasAccelerationLimits if true, applies acceleration limits.
    /// \param [in] _hasJerkLimits         if true, applies jerk limits.
    /// \param [in] _minVelocity Minimum velocity [m/s], usually <= 0.
    /// \param [in] _maxVelocity Maximum velocity [m/s], usually >= 0.
    /// \param [in] _minAcceleration Minimum acceleration [m/s^2], usually <= 0.
    /// \param [in] _maxAcceleration Maximum acceleration [m/s^2], usually >= 0.
    /// \param [in] _minJerk Minimum jerk [m/s^3], usually <= 0.
    /// \param [in] _maxJerk Maximum jerk [m/s^3], usually >= 0.
    public: SpeedLimiter(bool   _hasVelocityLimits     = false,
                         bool   _hasAccelerationLimits = false,
                         bool   _hasJerkLimits         = false,
                         double _minVelocity           = 0.0,
                         double _maxVelocity           = 0.0,
                         double _minAcceleration       = 0.0,
                         double _maxAcceleration       = 0.0,
                         double _minJerk               = 0.0,
                         double _maxJerk               = 0.0);

    /// \brief Destructor.
    public: ~SpeedLimiter();

    /// \brief Limit the velocity and acceleration.
    /// \param [in, out] _v  Velocity [m/s].
    /// \param [in]      _v0 Previous velocity to v  [m/s].
    /// \param [in]      _v1 Previous velocity to v0 [m/s].
    /// \param [in]      _dt Time step [s].
    /// \return Limiting factor (1.0 if none).
    public: double Limit(double &_v,
                         double _v0,
                         double _v1,
                         double _dt) const;

    /// \brief Limit the velocity.
    /// \param [in, out] _v Velocity [m/s].
    /// \return Limiting factor (1.0 if none).
    public: double LimitVelocity(double &_v) const;

    /// \brief Limit the acceleration.
    /// \param [in, out] _v  Velocity [m/s].
    /// \param [in]      _v0 Previous velocity [m/s].
    /// \param [in]      _dt Time step [s].
    /// \return Limiting factor (1.0 if none).
    public: double LimitAcceleration(double &_v,
                                     double _v0,
                                     double _dt) const;

    /// \brief Limit the jerk.
    /// \param [in, out] _v  Velocity [m/s].
    /// \param [in]      _v0 Previous velocity to v  [m/s].
    /// \param [in]      _v1 Previous velocity to v0 [m/s].
    /// \param [in]      _dt Time step [s].
    /// \return Limiting factor (1.0 if none).
    /// \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control.
    public: double LimitJerk(double &_v,
                             double _v0,
                             double _v1,
                             double _dt) const;

    /// \brief Private data pointer.
    private: std::unique_ptr<SpeedLimiterPrivate> dataPtr;
  };
  }
}
}
}

#endif
