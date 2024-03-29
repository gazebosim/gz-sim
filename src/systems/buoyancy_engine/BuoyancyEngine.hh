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
#ifndef GZ_SIM_SYSTEMS_BUOYANCYENGINE_HH_
#define GZ_SIM_SYSTEMS_BUOYANCYENGINE_HH_

#include <gz/sim/System.hh>

#include <memory>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class BuoyancyEnginePrivateData;

  /// \brief This class provides a simple mechanical bladder which is used to
  /// control the buoyancy of an underwater glider. It uses Archimedes'
  /// principle to apply an upward force based on the volume of the bladder. It
  /// listens to the topic `buoyancy_engine` or
  /// `/model/{namespace}/buoyancy_engine` topic for the volume of the bladder
  /// in *cubicmeters*.
  ///
  /// ## System Parameters
  /// - `<link_name>`: The link which the plugin is attached to [required,
  ///   string]
  /// - `<namespace>`: The namespace for the topic. If empty the plugin will
  ///   listen on `buoyancy_engine` otherwise it listens on
  ///   `/model/{namespace}/buoyancy_engine` [optional, string]
  /// - `<min_volume>`: Minimum volume of the engine [optional, float,
  ///   default=0.00003m^3]
  /// - `<neutral_volume>`: At this volume the engine has neutral buoyancy. Used
  ///   to estimate the weight of the engine [optional, float,
  ///   default=0.0003m^3]
  /// - `<default_volume>`: The volume which the engine starts at [optional,
  ///   float, default=0.0003m^3]
  /// - `<max_volume>`: Maximum volume of the engine [optional, float,
  ///   default=0.00099m^3]
  /// - `<max_inflation_rate>`: Maximum inflation rate for bladder [optional,
  ///   float, default=0.000003m^3/s]
  /// - `<fluid_density>`: The fluid density of the liquid its suspended in
  ///   kgm^-3. [optional, float, default=1000kgm^-3]
  /// - `<surface>`: The Z height in metres at which the surface of the water
  ///   is. If not defined then there is no surface [optional, float]
  ///
  /// ## Topics
  /// - Subscribes to a gz::msgs::Double on `buoyancy_engine` or
  ///   `/model/{namespace}/buoyancy_engine`. This is the set point for the
  ///   engine.
  /// - Publishes a gz::msgs::Double on `buoyancy_engine` or
  ///   `/model/{namespace}/buoyancy_engine/current_volume` on the current
  ///   volume
  ///
  /// ## Examples
  /// To get started run:
  /// ```
  /// gz sim buoyancy_engine.sdf
  /// ```
  /// Enter the following in a separate terminal:
  /** ```
      gz topic -t /model/buoyant_box/buoyancy_engine/ -m gz.msgs.Double \
         -p "data: 0.003"
      ```
  **/
  /// to see the box float up.
  /** ```
      gz topic -t /model/buoyant_box/buoyancy_engine/ -m gz.msgs.Double \
         -p "data: 0.001"
      ```
  **/
  /// to see the box go down.
  ///
  /// To see the current volume enter:
  /// ```
  /// gz topic -t /model/buoyant_box/buoyancy_engine/current_volume -e
  /// ```
  class BuoyancyEnginePlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: BuoyancyEnginePlugin();

    // Documentation inherited
    public: void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &/*_eventMgr*/
    );

    // Documentation inherited
    public: void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm);

    /// \brief Private data pointer
    private: std::unique_ptr<BuoyancyEnginePrivateData> dataPtr;
  };
}
}
}
}
#endif
