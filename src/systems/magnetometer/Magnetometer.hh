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
#ifndef IGNITION_GAZEBO_SYSTEMS_MAGNETOMETER_HH_
#define IGNITION_GAZEBO_SYSTEMS_MAGNETOMETER_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
namespace systems
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declarations.
  class MagnetometerPrivate;
  class MagnetometerSensor;

  /// \class Magnetometer Magnetometer.hh
  /// \brief An magnetometer sensor that reports vertical position and velocity
  /// readings over ign transport
  class IGNITION_GAZEBO_VISIBLE Magnetometer:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit Magnetometer();

    /// \brief Destructor
    public: virtual ~Magnetometer();

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override final;

    /// \brief Sets the rotation transform from world frame to IMU's
    /// reference frame.
    /// For example, if this IMU works with respect to NED frame, then
    /// call this function with the transform that transforms world frame
    /// to NED frame. Subsequently, MagnetometerSensor::Orientation will return
    /// identity transform if the IMU is aligned with the NED frame.
    /// \param _orientation rotation from world frame to magnetometer reference
    /// frame.
    public: void SetWorldToReferenceOrientation(
      const ignition::math::Quaterniond &_orientation);

    /// \brief Private data pointer.
    private: std::unique_ptr<MagnetometerPrivate> dataPtr;
  };
  }
}
}
}
#endif
