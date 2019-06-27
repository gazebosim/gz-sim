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
#ifndef IGNITION_MATH_ODOMETRYFOURWHEEL_HH_
#define IGNITION_MATH_ODOMETRYFOURWHEEL_HH_

#include <chrono>
#include <memory>
#include <ignition/math/Export.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Use a steady clock
    using clock = std::chrono::steady_clock;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    // Forward declarations.
    class OdometryFourWheelPrivate;

    /// \brief Computes odometry values based on a set of kinematic
    /// properties and wheel speeds for a four-wheeled vehicle.
    class OdometryFourWheel
    {
      /// \brief Constructor.
      /// \param[in] _windowSize Rolling window size used to compute the
      /// velocity mean
      public: OdometryFourWheel(size_t _windowSize = 10);

      /// \brief Initialize the odometry
      /// \param[in] _time Current time.
      public: void Init(const clock::time_point &_time);

      /// \brief Updates the odometry class with latest wheels and
      /// steerings position
      /// \param[in] _frontLeftSpeed Front left wheel vehicle speed [rad/s]
      /// \param[in] _frontRightSpeed Front right wheel vehicle speed [rad/s]
      /// \param[in] _rearLeftSpeed Rear left wheel vehicle speed [rad/s]
      /// \param[in] _rearRightSpeed Rear right wheel vehicle speed [rad/s]
      /// \param[in] _frontSteering Front steering position [rad]
      /// \param[in] _rearSteering Rear steering position [rad]
      /// \param[in] _time Current time
      /// \return True if the odometry is actually updated.
      public: bool Update(double _frontLeftSpeed,
                          double _frontRightSpeed,
                          double _rearLeftSpeed,
                          double _rearRightSpeed,
                          double _frontSteering,
                          double _rearSteering,
                          const clock::time_point &_time);

      /// \brief Get the heading.
      /// \return The heading in radians.
      public: double Heading() const;

      /// \brief Get the X position.
      ///  \return The X position in meters
      public: double X() const;

      /// \brief Get the Y position.
      /// \return The Y position in meters.
      public: double Y() const;

      /// \brief Get the linear velocity.
      /// \return The linear velocity in meter/second.
      public: double LinearVelocity() const;

      /// \brief Get the linear velocity along the local X axis.
      /// \return The linear velocity in meter/second.
      public: double LinearVelocityX() const;

      /// \brief Get the linear velocity along the local Y axis.
      /// \return The linear velocity meter/second.
      public: double LinearVelocityY() const;

      /// \brief Get the angular velocity.
      /// \return The angular velocity in radian/second.
      public: double AngularVelocity() const;

      /// \brief Get the linear acceleration.
      /// \return The linear acceleration in meter/(second^2)
      public: double LinearAcceleration() const;

      /// \brief Get the linear jerk.
      /// \return The linear jerk in meter/(second^3).
      public: double LinearJerk() const;

      /// \brief Get the front steering velocity.
      /// \return The front steering velocity in meter/(second^3)
      public: double FrontSteerVelocity() const;

      /// \brief Get the rear steering velocity
      /// \return The rear steer velocity in meter/(second^3)
      public: double RearSteerVelocity() const;

      /// \brief Set the wheel parameters including the radius and separation.
      /// \param[in] _steeringTrack Seperation between left and right steering
      /// joints in meters.
      /// \param[in] _wheelSteeringYOffset Offset between the steering and
      /// wheel joints in meters.
      /// \param[in] _radius Wheel radius in meters.
      /// \param[in] _base Wheel base in meters.
      public: void SetWheelParams(double _steeringTrack,
                  double _wheelSteeringYOffset,
                  double _radius, double _base);

      /// \brief Set the velocity rolling window size.
      /// \param[in] _size The Velocity rolling window size.
      public: void SetVelocityRollingWindowSize(size_t _size);

      /// \brief Private data pointer.
      private: std::unique_ptr<OdometryFourWheelPrivate> dataPtr;
    };
    }
  }
}

#endif
