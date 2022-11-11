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
#ifndef GZ_SIM_SYSTEMS_ENVIRONMENTALDATAPRELOAD_HH_
#define GZ_SIM_SYSTEMS_ENVIRONMENTALDATAPRELOAD_HH_

#include <memory>

#include "gz/sim/config.hh"
#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \class EnvironmentalDataPreload EnvironmentalDataPreload.hh
  ///     gz/sim/systems/EnvironmentalDataPreload.hh
  /// \brief A plugin to preload an EnvironmentalData component
  /// into the ECM upon simulation start-up.
  class EnvironmentalDataPreload :
    public System,
    public ISystemConfigure
  {
    /// \brief Constructor
    public: explicit EnvironmentalDataPreload();

    /// \brief Destructor
    public: ~EnvironmentalDataPreload() override;

    // Documentation inherited
    public: void Configure(
        const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        sim::EventManager &_eventMgr) final;
  };
  }
}
}
}
#endif
