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
#ifndef GZ_GAZEBO_SYSTEMMANAGER_HH_
#define GZ_GAZEBO_SYSTEMMANAGER_HH_

#include <memory>
#include <string>
#include <vector>

#include "gz/sim/config.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"

#include "SystemInternal.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {

    /// \brief Used to load / unload sysetms as well as iterate over them.
    class GZ_GAZEBO_VISIBLE SystemManager
    {
      /// \brief Constructor
      /// \param[in] _systemLoader A pointer to a SystemLoader to load plugins
      ///  from files
      /// \param[in] _entityCompMgr Pointer to the entity component manager to
      ///  be used when configuring new systems
      /// \param[in] _eventMgr Pointer to the event manager to be used when
      ///  configuring new systems
      public: explicit SystemManager(const SystemLoaderPtr &_systemLoader,
                            EntityComponentManager *_entityCompMgr = nullptr,
                            EventManager *_eventMgr = nullptr);

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

      /// \brief Move all "pending" systems to "active" state
      /// \return The number of newly-active systems
      public: size_t ActivatePendingSystems();

      /// \brief Get an vector of all active systems implementing "Configure"
      /// \return Vector of systems's configure interfaces.
      public: const std::vector<ISystemConfigure *>& SystemsConfigure();

      /// \brief Get an vector of all active systems implementing "Reset"
      /// \return Vector of systems's reset interfaces.
      public: const std::vector<ISystemReset *>& SystemsReset();

      /// \brief Get an vector of all active systems implementing "PreUpdate"
      /// \return Vector of systems's pre-update interfaces.
      public: const std::vector<ISystemPreUpdate *>& SystemsPreUpdate();

      /// \brief Get an vector of all active systems implementing "Update"
      /// \return Vector of systems's update interfaces.
      public: const std::vector<ISystemUpdate *>& SystemsUpdate();

      /// \brief Get an vector of all active systems implementing "PostUpdate"
      /// \return Vector of systems's post-update interfaces.
      public: const std::vector<ISystemPostUpdate *>& SystemsPostUpdate();

      /// \brief Get an vector of all systems attached to a given entity.
      /// \return Vector of systems.
      public: std::vector<SystemInternal> TotalByEntity(Entity _entity);

      /// \brief Implementation for AddSystem functions. This only adds systems
      /// to a queue, the actual addition is performed by `AddSystemToRunner` at
      /// the appropriate time.
      /// \param[in] _system Generic representation of a system.
      /// \param[in] _sdf SDF received from AddSystem.
      private: void AddSystemImpl(SystemInternal _system,
                                  std::shared_ptr<const sdf::Element> _sdf);

      /// \brief All the systems.
      private: std::vector<SystemInternal> systems;

      /// \brief Pending systems to be added to systems.
      private: std::vector<SystemInternal> pendingSystems;

      /// \brief Mutex to protect pendingSystems
      private: mutable std::mutex pendingSystemsMutex;

      /// \brief Systems implementing Configure
      private: std::vector<ISystemConfigure *> systemsConfigure;

      /// \brief Systems implementing Reset
      private: std::vector<ISystemReset *> systemsReset;

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

      /// \brief Pointer to associated entity component manager
      private: EntityComponentManager *entityCompMgr;

      /// \brief Pointer to associated event manager
      private: EventManager *eventMgr;
    };
    }
  }  // namespace sim
}  // namespace gz
#endif  // GZ_GAZEBO_SYSTEMINTERNAL_HH_
