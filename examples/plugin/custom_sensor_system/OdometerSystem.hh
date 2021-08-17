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
#ifndef ODOMETERSYSTEM_HH_
#define ODOMETERSYSTEM_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/sensors/Sensor.hh>
#include <ignition/transport/Node.hh>

namespace custom
{
  /// \brief Example showing how to tie a custom sensor, in this case an
  /// odometer, into simulation
  class OdometerSystem:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
  {
    // Documentation inherited.
    // During PreUpdate, check for new sensors that were inserted
    // into simulation and create more components as needed.
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) final;

    // Documentation inherited.
    // During PostUpdate, update the known sensors and publish their data.
    // Also remove sensors that have been deleted.
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
        const ignition::gazebo::EntityComponentManager &_ecm) final;

    /// \brief Remove custom sensors if their entities have been removed from
    /// simulation.
    /// \param[in] _ecm Immutable reference to ECM.
    private: void RemoveSensorEntities(
        const ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief A map of custom entities to their sensors
    private: std::unordered_map<ignition::gazebo::Entity,
        std::shared_ptr<Odometer>> entitySensorMap;
  };
}
#endif
