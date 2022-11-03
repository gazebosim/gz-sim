/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_ENVIRONMENTAL_SYSTEM_HH_
#define GZ_ENVIRONMENTAL_SYSTEM_HH_

#include <gz/sim/System.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>

#include <memory>

namespace gz
{
namespace sim
{
class EnvironmentalSensorSystemPrivate;
/// \brief Sensor for reading environmental data loaded from outside world.
/// To use add this system to the world file, then instantiate sensors of custom
/// type with gz:type="environmental_sensor/{field_name}". Where field_name
/// refers to the type of data you would like to output.
/// Alternatively, if you would like to specify a custom field name you may do
/// so using the <environment_variable> tag.
class EnvironmentalSensorSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  public: EnvironmentalSensorSystem();
  /// Documentation inherited
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/) override;

  // Documentation inherited.
  // During PreUpdate, check for new sensors that were inserted
  // into simulation and create more components as needed.
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) final;

  // Documentation inherited.
  // During PostUpdate, update the known sensors and publish their data.
  // Also remove sensors that have been deleted.
  public: void PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) final;

  private: std::unique_ptr<EnvironmentalSensorSystemPrivate> dataPtr;
};
}
}

#endif
