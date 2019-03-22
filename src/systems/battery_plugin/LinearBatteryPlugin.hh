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

#ifndef IGNITION_GAZEBO_SYSTEMS_LINEAR_BATTERY_PLUGIN_HH_
#define IGNITION_GAZEBO_SYSTEMS_LINEAR_BATTERY_PLUGIN_HH_

#include <string>
#include <map>
#include <memory>

#include <ignition/common/Battery.hh>

#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class LinearBatteryPluginPrivate;

  /// \brief A plugin that simulates a linear battery.
  class IGNITION_GAZEBO_VISIBLE LinearBatteryPlugin
      : public System,
        public ISystemConfigure,
        public ISystemUpdate
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
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) final;

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
