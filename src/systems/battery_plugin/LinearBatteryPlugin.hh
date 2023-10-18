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

#ifndef GZ_SIM_SYSTEMS_LINEAR_BATTERY_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_LINEAR_BATTERY_PLUGIN_HH_

#include <string>
#include <map>
#include <memory>

#include <gz/common/Battery.hh>

#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class LinearBatteryPluginPrivate;

  /// \brief A plugin for simulating battery usage
  ///
  /// ## System parameters
  ///
  /// - `<battery_name>` name of the battery (required)
  /// - `<voltage>` Initial voltage of the battery (required)
  /// - `<open_circuit_voltage_constant_coef>` Voltage at full charge
  /// - `<open_circuit_voltage_linear_coef>` Amount of voltage decrease when no
  ///   charge
  /// - `<initial_charge>` Initial charge of the battery (Ah)
  /// - `<capacity>` Total charge that the battery can hold (Ah)
  /// - `<resistance>` Internal resistance (Ohm)
  /// - `<smooth_current_tau>` coefficient for smoothing current [0, 1].
  /// - `<power_load>` power load on battery (required) (Watts)
  /// - `<enable_recharge>` If true, the battery can be recharged
  /// - `<recharge_by_topic>` If true, the start/stop signals for recharging the
  ///   battery will also be available via topics. The
  ///   regular Gazebo services will still be available.
  /// - `<charging_time>` Hours taken to fully charge the battery.
  ///   (Required if `<enable_recharge>` is set to true)
  /// - `<fix_issue_225>` True to change the battery behavior to fix some issues
  ///   described in https://github.com/gazebosim/gz-sim/issues/225.
  /// - `<start_drainign>` Whether to start draining the battery right away.
  ///   False by default.
  /// - `<power_draining_topic>` A topic that is used to start battery
  ///   discharge. Any message on the specified topic will cause the battery to
  ///   start draining. This element can be specified multiple times if
  ///   multiple topics should be monitored. Note that this mechanism will
  ///   start the battery draining, and once started will keep drainig.
  /// - `<stop_power_draining_topic>` A topic that is used to stop battery
  ///   discharge. Any message on the specified topic will cause the battery to
  ///   stop draining.

  class LinearBatteryPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: LinearBatteryPlugin();

    /// \brief Destructor
    public: ~LinearBatteryPlugin() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// Documentation inherited
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Callback for Battery Update events.
    /// \param[in] _battery Pointer to the battery that is to be updated.
    /// \return The new voltage.
    private: double OnUpdateVoltage(const common::Battery *_battery);

    /// \brief Private data pointer
    private: std::unique_ptr<LinearBatteryPluginPrivate> dataPtr;
  };
  }
}
}
}

#endif
