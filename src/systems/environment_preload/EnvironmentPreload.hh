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
#ifndef GZ_SIM_SYSTEMS_ENVIRONMENTPRELOAD_HH_
#define GZ_SIM_SYSTEMS_ENVIRONMENTPRELOAD_HH_

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
  class EnvironmentPreloadPrivate;

  /** \class EnvironmentPreload EnvironmentPreload.hh \
   * gz/sim/systems/EnvironmentPreload.hh
  **/
  /// \brief A plugin to preload an Environment component
  /// into the ECM upon simulation start-up.
  class EnvironmentPreload :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    /// \brief Constructor
    public: explicit EnvironmentPreload();

    /// \brief Destructor
    public: ~EnvironmentPreload() override;

    // Documentation inherited
    public: void Configure(
        const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        sim::EventManager &_eventMgr) final;

    // Documentation inherited
    public: void PreUpdate(
        const UpdateInfo &_info,
        EntityComponentManager &_ecm) final;

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<EnvironmentPreloadPrivate> dataPtr;
  };
  }
}
}
}
#endif
