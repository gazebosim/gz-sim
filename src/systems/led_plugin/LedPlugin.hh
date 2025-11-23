/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#ifndef GZ_SIM_LED_PLUGIN_SYSTEM_HH_
#define GZ_SIM_LED_PLUGIN_SYSTEM_HH_

#include <memory>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief Forward declare private data class.
  class LedPluginPrivate;

  /** \class LedPlugin LedPlugin.hh \
   * gz/sim/systems/LedPlugin.hh
  **/
  /// \brief Plugin that makes a visual blink between two colors. This can be used to simulate
  /// LEDs .See the example usage below:
  ///
  /// ## System Parameters:
  ///
  /// <plugin name="led" filename="libLedPlugin.so">
  ///
  ///   <led_name>name</led_name>
  ///   <default_mode>mode_name</default_mode>
  ///   <create_light>true/false</create_light>
  ///   <light>
  ///     .
  ///     .
  ///     .
  ///   </light>
  ///   <led_modes>
  ///     <mode name=”mode_name1”>
  ///       <step always_on=”false”>
  ///         <color>r g b a</color>
  ///         <on_time>time(s)<on_time>
  ///       </step>
  ///       <step always_on=”false”>
  ///         <color>r g b a</color>
  ///         <on_time>time(s)<on_time>
  ///       </step>
  ///       <step always_on=”false”>
  ///         <color>r g b a</color>
  ///         <on_time>time(s)<on_time>
  ///       </step>
  ///     </mode>
  ///     <mode name=”mode_name2”>
  ///       <step always_on=”true”>
  ///         <color>r g b a</color>
  ///       </step>
  ///     </mode>
  ///     <mode name=”mode_name3”>
  /// 	.
  /// 	.
  /// 	.
  ///     </mode>
  ///   </led_modes>
  /// </plugin>

  class LedPlugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    /// \brief Constructor.
    public: LedPlugin();

    /// \brief Destructor.
    public: ~LedPlugin() override;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<LedPluginPrivate> dataPtr;
  };
}
}
}
}
#endif