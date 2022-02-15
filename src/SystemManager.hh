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
#ifndef IGNITION_GAZEBO_SYSTEMMANAGER_HH_
#define IGNITION_GAZEBO_SYSTEMMANAGER_HH_

#include <memory>
#include <string>
#include <vector>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/Types.hh"

#include "SystemInternal.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

    /// \brief Used to load / unload sysetms as well as iterate over them.
    class SystemManager
    {
      /// \brief Constructor
      /// \param[in] _systemLoader A pointer to a SystemLoader to load plugins
      ///  from files
      public: explicit SystemManager(const SystemLoaderPtr &_systemLoader);

      /// \brief Load system plugin for a given entity.
      /// \param[in] _entity Entity
      /// \param[in] _fname Filename of the plugin library
      /// \param[in] _name Name of the plugin
      /// \param[in] _sdf SDF element (content of plugin tag)
      public: void LoadPlugin(const Entity _entity,
                              const std::string &_fname,
                              const std::string &_name,
                              const sdf::ElementPtr &_sdf);

      /// \brief Add a system to the manager
      /// \param[in] _system SystemPluginPtr to be added
      /// \param[in] _entity Entity that system is attached to.
      /// \param[in] _sdf Pointer to the SDF of the entity.
      public: void AddSystem(const SystemPluginPtr &_system,
                             Entity _entity,
                             std::shared_ptr<const sdf::Element> _sdf);

      /// \brief Add a system to the manager
      /// \param[in] _system SystemPluginPtr to be added
      /// \param[in] _entity Entity that system is attached to.
      /// \param[in] _sdf Pointer to the SDF of the entity.
      public: void AddSystem(const std::shared_ptr<System> &_system,
                             Entity _entity,
                             std::shared_ptr<const sdf::Element> _sdf);

      /// \brief Get the count of currently active systems.
      /// \return The active systems count.
      public: size_t ActiveCount() const;

      /// \brief Get the count of currently pending systems.
      /// \return The pending systems count.
      public: size_t PendingCount() const;

      /// \brief Get the count of all (pending + active) managed systems
      /// \return The count.
      public: size_t TotalCount() const;

      /// \brief Call the configure call on all pending systems
      /// \param[in] _ecm The entity component manager to configure with
      /// \param[in] _evetnMgr The event manager to configure with
      public: void ConfigurePendingSystems(EntityComponentManager &_ecm,
                                           EventManager &_eventMgr);

      /// \brief Move all "pending" systems to "active" state
      /// \return The number of newly-active systems
      public: size_t ActivatePendingSystems();

      /// \brief Get an vector of all systems implementing "Configure"
      public: const std::vector<ISystemConfigure *>& SystemsConfigure();

      /// \brief Get an vector of all systems implementing "PreUpdate"
      public: const std::vector<ISystemPreUpdate *>& SystemsPreUpdate();

      /// \brief Get an vector of all systems implementing "Update"
      public: const std::vector<ISystemUpdate *>& SystemsUpdate();

      /// \brief Get an vector of all systems implementing "PostUpdate"
      public: const std::vector<ISystemPostUpdate *>& SystemsPostUpdate();

      /// \brief Implementation for AddSystem functions. This only adds systems
      /// to a queue, the actual addition is performed by `AddSystemToRunner` at
      /// the appropriate time.
      /// \param[in] _system Generic representation of a system.
      /// \param[in] _entity Entity received from AddSystem.
      /// \param[in] _sdf SDF received from AddSystem.
      private: void AddSystemImpl(SystemInternal _system,
                                  Entity _entity,
                                  std::shared_ptr<const sdf::Element> _sdf);

      /// \brief All the systems.
      private: std::vector<SystemInternal> systems;

      /// \brief Pending systems to be added to systems.
      private: std::vector<SystemInternal> pendingSystems;

      /// \brief Mark if a pending system has been configured
      private: std::vector<bool> pendingSystemsConfigured;

      /// \brief Mutex to protect pendingSystems
      private: mutable std::mutex systemsMutex;

      /// \brief Systems implementing Configure
      private: std::vector<ISystemConfigure *> systemsConfigure;

      /// \brief Systems implementing PreUpdate
      private: std::vector<ISystemPreUpdate *> systemsPreupdate;

      /// \brief Systems implementing Update
      private: std::vector<ISystemUpdate *> systemsUpdate;

      /// \brief Systems implementing PostUpdate
      private: std::vector<ISystemPostUpdate *> systemsPostupdate;

      /// \brief System loader, for loading system plugins.
      private: SystemLoaderPtr systemLoader;

      /// \brief Mutex to protect systemLoader
      private: std::mutex systemLoaderMutex;
    };
    }
  }  // namespace gazebo
}  // namespace ignition
#endif  // IGNITION_GAZEBO_SYSTEMINTERNAL_HH_
