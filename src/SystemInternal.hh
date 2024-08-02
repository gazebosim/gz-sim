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
#ifndef GZ_SIM_SYSTEMINTERNAL_HH_
#define GZ_SIM_SYSTEMINTERNAL_HH_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gz/sim/config.hh"
#include "gz/sim/System.hh"
#include "gz/sim/SystemPluginPtr.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {

    /// \brief Class to hold systems internally. It supports systems loaded
    /// from plugins, as well as systems created at runtime.
    class SystemInternal
    {
      /// \brief Constructor
      /// \param[in] _systemPlugin A system loaded from a plugin.
      /// \param[in] _entity The entity that this system is attached to.
      public: explicit SystemInternal(SystemPluginPtr _systemPlugin,
          Entity _entity)
              : systemPlugin(std::move(_systemPlugin)),
                system(systemPlugin->QueryInterface<System>()),
                configure(systemPlugin->QueryInterface<ISystemConfigure>()),
                configureParameters(
                  systemPlugin->QueryInterface<ISystemConfigureParameters>()),
                configurePriority(
                  systemPlugin->QueryInterface<ISystemConfigurePriority>()),
                reset(systemPlugin->QueryInterface<ISystemReset>()),
                preupdate(systemPlugin->QueryInterface<ISystemPreUpdate>()),
                update(systemPlugin->QueryInterface<ISystemUpdate>()),
                postupdate(systemPlugin->QueryInterface<ISystemPostUpdate>()),
                parentEntity(_entity)
      {
      }

      /// \brief Constructor
      /// \param[in] _system Pointer to a system.
      /// \param[in] _entity The entity that this system is attached to.
      public: explicit SystemInternal(const std::shared_ptr<System> &_system,
          Entity _entity)
              : systemShared(_system),
                system(_system.get()),
                configure(dynamic_cast<ISystemConfigure *>(_system.get())),
                configureParameters(
                  dynamic_cast<ISystemConfigureParameters *>(_system.get())),
                configurePriority(
                  dynamic_cast<ISystemConfigurePriority *>(_system.get())),
                reset(dynamic_cast<ISystemReset *>(_system.get())),
                preupdate(dynamic_cast<ISystemPreUpdate *>(_system.get())),
                update(dynamic_cast<ISystemUpdate *>(_system.get())),
                postupdate(dynamic_cast<ISystemPostUpdate *>(_system.get())),
                parentEntity(_entity)
      {
      }

      /// \brief Plugin object. This manages the lifecycle of the instantiated
      /// class as well as the shared library.
      /// This will be null if the system wasn't loaded from a plugin.
      public: SystemPluginPtr systemPlugin;

      /// \brief Pointer to a system.
      /// This will be null if the system wasn't loaded from a pointer.
      public: std::shared_ptr<System> systemShared{nullptr};

      /// \brief Access this system via the `System` interface
      public: System *system = nullptr;

      /// \brief Access this system via the ISystemConfigure interface
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemConfigure *configure = nullptr;

      /// \brief Access this system via the ISystemConfigureParameters
      ///   interface.
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemConfigureParameters *configureParameters = nullptr;

      /// \brief Access this system via the ISystemConfigurePriority
      ///   interface.
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemConfigurePriority *configurePriority = nullptr;

      /// \brief Access this system via the ISystemReset interface
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemReset *reset = nullptr;

      /// \brief Access this system via the ISystemPreUpdate interface
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemPreUpdate *preupdate = nullptr;

      /// \brief Access this system via the ISystemUpdate interface
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemUpdate *update = nullptr;

      /// \brief Access this system via the ISystemPostUpdate interface
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemPostUpdate *postupdate = nullptr;

      /// \brief Entity that the system is attached to. It's passed to the
      /// system during the `Configure` call.
      public: Entity parentEntity = {kNullEntity};

      /// \brief Cached filename of the plugin used when system was loaded.
      /// Used for reloading a system at runtime.
      public: std::string fname = "";

      /// \brief Cached plugin name of the plugin used when system was loaded.
      /// Used for reloading a system at runtime.
      public: std::string name = "";

      /// \brief Cached sdf that was used to call `Configure` on the system
      /// Useful for if a system needs to be reconfigured at runtime
      public: std::shared_ptr<const sdf::Element> configureSdf = nullptr;

      /// \brief Vector of queries and callbacks
      public: std::vector<EntityQueryCallback> updates;
    };
    }
  }  // namespace sim
}  // namespace gz
#endif  // GZ_SIM_SYSTEMINTERNAL_HH_
