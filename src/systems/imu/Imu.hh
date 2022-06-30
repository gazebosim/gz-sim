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
#ifndef GZ_SIM_SYSTEMS_IMU_HH_
#define GZ_SIM_SYSTEMS_IMU_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class ImuPrivate;

  /// \class Imu Imu.hh gz/sim/systems/Imu.hh
  /// \brief This system manages all IMU sensors in simulation.
  /// Each IMU sensor eports vertical position, angular velocity
  /// and lienar acceleration readings over Gazebo Transport.
  class Imu:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit Imu();

    /// \brief Destructor
    public: ~Imu() override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<ImuPrivate> dataPtr;
  };
  }
}
}
}
#endif
