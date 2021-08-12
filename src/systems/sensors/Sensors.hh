/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_SENSORS_HH_
#define IGNITION_GAZEBO_SYSTEMS_SENSORS_HH_

#include <memory>
#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/System.hh>
#include <sdf/Sensor.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class SensorsPrivate;

  /// \class Sensors Sensors.hh ignition/gazebo/systems/Sensors.hh
  /// \brief TODO(louise) Have one system for all sensors, or one per
  /// sensor / sensor type?
  class Sensors:
    public System,
    public ISystemConfigure,
    public ISystemUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit Sensors();

    /// \brief Destructor
    public: ~Sensors() override;

    // Documentation inherited
    public: void Configure(const Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Create a rendering sensor from sdf
    /// \param[in] _entity Entity of the sensor
    /// \param[in] _sdf SDF description of the sensor
    /// \param[in] _parentName Name of parent that the sensor is attached to
    /// \return Sensor name
    private : std::string CreateSensor(const Entity &_entity,
                                       const sdf::Sensor &_sdf,
                                       const std::string &_parentName);

    /// \brief Removes a rendering sensor
    /// \param[in] _entity Entity of the sensor
    private : void RemoveSensor(const Entity &_entity);

    /// \brief Private data pointer.
    private: std::unique_ptr<SensorsPrivate> dataPtr;
  };
  }
}
}
}
#endif
