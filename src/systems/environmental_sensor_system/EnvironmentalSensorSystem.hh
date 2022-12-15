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
/// To use, add this system to the world file, then instantiate sensors of
/// custom type with gz:type="environmental_sensor/{field_name}", where
/// field_name refers to the type of data you would like to output.
/// Alternatively, if you would like to specify a custom field name you may do
/// so using the <environment_variable> tag.
///
/// Additionally, the environment sensor supports scenarios where the data is in
/// the form of vector fields. For instance in the case of wind or ocean
/// currents.
///
/// Tags:
///  <output_format> - Either "scalar" or "vector3" depending on type
///    of output data desired.
///  <environment_variable> - Only for scalar type. The name of the column of
///    the CSV file to be used.
///  <environment_variable_x> - The name of the field to be used as the x value
///    in global frame for vector3 fields. Note: If this is left out and
///    vector3 is set, then the x value defaults to zero.
///  <environment_variable_y> - The name of the field to be used as the y value
///    in global frame for vector3 fields. Note: If this is left out and
///    vector3 is set, then the y value defaults to zero.
///  <environment_variable_z> - The name of the field to be used as the z value
///    in global frame vector3 fields. Note: If this is left out and
///    vector3 is set, then the z value defaults to zero.
///  <transform_type> - When handling vector2 or vector3 types it may be in our
///    interest to transform them to local coordinate frames. For instance
///    measurements of ocean currents or wind depend on the orientation and
///    velocity of the sensor. This field can have 4 values:
///      * GLOBAL - Don't transform the vectors to a local frame.
///      * LOCAL - Transform the vector to a local frame.
///      * ADD_VELOCITY_GLOBAL - Don't transform to local frame but account for
///          velocity change.
///      * ADD_VELOCITY_LOCAL - Transform to local frame and account for sensor
///          velocity. If you're working with wind or ocean currents, this is
///          probably the option you want.
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
