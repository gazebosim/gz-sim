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
#ifndef IGNITION_GAZEBO_SYSTEMS_AIRPRESSURE_HH_
#define IGNITION_GAZEBO_SYSTEMS_AIRPRESSURE_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/air-pressure-system/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class AirPressurePrivate;

  /// \class AirPressure AirPressure.hh ignition/gazebo/systems/AirPressure.hh
  /// \brief An air pressure sensor that reports vertical position and velocity
  /// readings over ign transport
  class IGNITION_GAZEBO_AIR_PRESSURE_SYSTEM_VISIBLE AirPressure:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit AirPressure();

    /// \brief Destructor
    public: ~AirPressure() override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;


    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<AirPressurePrivate> dataPtr;
  };
  }
}
}
}
#endif
