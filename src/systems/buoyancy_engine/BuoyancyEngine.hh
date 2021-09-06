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
#ifndef IGNITION_GAZEBO_SYSTEMS_BALLAST_HH_
#define IGNITION_GAZEBO_SYSTEMS_BALLAST_HH_

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  class BuoyancyEnginePrivateData;

  /// \brief This class provides a simple mechanical bladder which is used to
  /// control the buoyancy of an underwater glider. It uses archimedes principal
  /// to apply an upward force based on the volume of the bladder. It listens to
  /// the topic `buoyancy_engine` or `/model/{namespace}/buoyancy_engine`
  /// topic for the volume of the bladder in *cubicentimers*.
  ///
  /// ## Parameters
  /// <link_name> - The link which the plugin is attached to [required, string]
  /// <namespace> - The namespace for the topic. If empty the plugin will listen
  ///   on `buoyancy_engine` otherwise it listens on
  ///   `/model/{namespace}/buoyancy_engine` [optional, string]
  /// <min_volume> - Minimum volume of the engine [optional, float,
  ///   default=0.00003m^3]
  /// <neutral_volume> - At this volume the engine has neutral buoyancy. Used to
  ///   estimate the weight of the engine [optional, float, default=0.0003m^3]
  /// <default_volume> - The volume which the engine starts at [optional, float,
  ///   default=0.0003m^3]
  /// <max_volume> - Maximum volume of the engine [optional, float,
  ///   default=0.00099m^3]
  /// <max_inflation_rate> - Maximum inflation rate for bladder [optional,
  ///   float, default=0.000003m^3/s]
  ///
  /// ## Topics
  /// * Subscribes to a ignition::msgs::Double on `buoyancy_engine` or
  ///  `/model/{namespace}/buoyancy_engine`. This is the set point for the
  ///  engine.
  /// * Publishes a ignition::msgs::Double on `buoyancy_engine` or
  ///  `/model/{namespace}/buoyancy_engine` on the currents
  class BuoyancyEnginePlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    public: BuoyancyEnginePlugin();

    /// Inherits documentation from parent class
    public: void Configure(
        const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &/*_eventMgr*/
    );

    /// Inherits documentation from parent class
    public: void PreUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm);

    /// Inherits documentation from parent class
    private: std::unique_ptr<BuoyancyEnginePrivateData> dataPtr;
  };
}
}
}
}
#endif