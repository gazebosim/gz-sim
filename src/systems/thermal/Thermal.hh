/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_THERMAL_HH_
#define IGNITION_GAZEBO_SYSTEMS_THERMAL_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
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
  class ThermalPrivate;

  /// \brief A thermal plugin that sets the temperature for the parent entity
  class IGNITION_GAZEBO_VISIBLE Thermal:
    public System,
    public ISystemConfigure
  {
    /// \brief Constructor
    public: explicit Thermal();

    /// \brief Destructor
    public: ~Thermal() override;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           gazebo::EventManager &_eventMgr) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<ThermalPrivate> dataPtr;
  };
  }
}
}
}
#endif
