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
#ifndef GZ_SIM_SYSTEMMANAGER_HH_
#define GZ_SIM_SYSTEMMANAGER_HH_

#include <gz/msgs/entity_plugin_v.pb.h>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <sdf/Plugin.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/config.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/System.hh"
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
    class GZ_SIM_VISIBLE SystemManager
    {
      /// \brief Ordered map of priority values to a vector of System
      /// interfaces.
      using PriorityType = System::PriorityType;
      template<typename S>
      class PrioritizedSystems : public std::map<PriorityType, std::vector<S>>
      {};

      /// \brief Constructor
      /// \param[in] _systemLoader A pointer to a SystemLoader to load plugins
      ///  from files
      /// \param[in] _entityCompMgr Pointer to the entity component manager to
      ///  be used when configuring new systems
      /// \param[in] _eventMgr Pointer to the event manager to be used when
      ///  configuring new systems
      /// \param[in] _namespace Namespace to use for the transport node
      public: explicit SystemManager(
        const SystemLoaderPtr &_systemLoader,
        EntityComponentManager *_entityCompMgr = nullptr,
        EventManager *_eventMgr = nullptr,
        const std::string &_namespace = std::string(),
        gz::transport::parameters::ParametersRegistry *
          _parametersRegistry = nullptr);

      /// \brief Load system plugin for a given entity.
      /// \param[in] _entity Entity
      /// \param[in] _plugin Plugin to load
      public: void LoadPlugin(const Entity _entity,
                              const sdf::Plugin &_plugin);

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

      /// \brief Perform a reset on all systems
      ///
      /// If a system implements the ISystemReset interface, it will be called.
      //
      /// Otherwise, if a system does not have the ISystemReset interface
      /// implemented, and was created via loading a plugin,
      /// that plugin will be reloaded.
      ///
      /// Otherwise, if a system is created from in-memory rather than a plugin,
      /// that system will remain unaffected.
      /// \param[in] _info Update info corresponding to the update time
      /// \param[in] _ecm Version of the ECM reset to an initial state
      public: void Reset(const UpdateInfo &_info, EntityComponentManager &_ecm);

      /// \brief Get a vector of all systems implementing "Configure"
      /// \return Vector of systems' configure interfaces.
      public: const std::vector<ISystemConfigure *>& SystemsConfigure();

      /// \brief Get a vector of all active systems implementing
      ///   "ConfigureParameters"
      /// \return Vector of systems's configure interfaces.
      public: const std::vector<ISystemConfigureParameters *>&
      SystemsConfigureParameters();

      /// \brief Get a vector of all active systems implementing "Reset"
      /// \return Vector of systems' reset interfaces.
      public: const std::vector<ISystemReset *>& SystemsReset();

      /// \brief Get an ordered map of systems by priority that implement
      /// "PreUpdate"
      /// \return Priortized map of systems's pre-update interfaces.
      public: const PrioritizedSystems<ISystemPreUpdate *>& SystemsPreUpdate();

      /// \brief Get an ordered map of systems by priority that implement
      /// "Update"
      /// \return Priortized map of systems's update interfaces.
      public: const PrioritizedSystems<ISystemUpdate *>& SystemsUpdate();

      /// \brief Get a vector of all active systems implementing "PostUpdate"
      /// \return Vector of systems's post-update interfaces.
      public: const std::vector<ISystemPostUpdate *>& SystemsPostUpdate();

      /// \brief Get a vector of all systems attached to a given entity.
      /// \return Vector of systems.
      public: std::vector<SystemInternal> TotalByEntity(Entity _entity);

      /// \brief Process system messages and add systems to entities
      public: void ProcessPendingEntitySystems();

      /// \brief Remove systems that are attached to removed entities
      /// \param[in] _entityCompMgr - ECM with entities marked for removal
      /// \param[out] _needsCleanUp - Set to true if a system with a
      /// PostUpdate was removed, and its thread needs to be terminated
      public: void ProcessRemovedEntities(
        const EntityComponentManager &_entityCompMgr,
        bool &_needsCleanUp);

      /// \brief Implementation for AddSystem functions that takes an SDF
      /// element. This calls the AddSystemImpl that accepts an SDF Plugin.
      /// \param[in] _system Generic representation of a system.
      /// \param[in] _sdf SDF element.
      private: void AddSystemImpl(SystemInternal _system,
                                  std::shared_ptr<const sdf::Element> _sdf);

      /// \brief Implementation for AddSystem functions. This only adds systems
      /// to a queue, the actual addition is performed by `AddSystemToRunner` at
      /// the appropriate time.
      /// \param[in] _system Generic representation of a system.
      /// \param[in] _sdf SDF received from AddSystem.
      private: void AddSystemImpl(SystemInternal _system,
                                  const sdf::Plugin &_sdf);

      /// \brief Callback for entity add system service.
      /// \param[in] _req Request message containing the entity id and plugins
      /// to add to that entity
      /// \param[out] _res Response containing a boolean value indicating if
      /// service request is received or not
      /// \return True if request received.
      private: bool EntitySystemAddService(const msgs::EntityPlugin_V &_req,
                                           msgs::Boolean &_res);

      /// \brief Callback for entity info system service.
      /// \param[in] _req Empty request message
      /// \param[out] _res Response containing a list of plugin names
      /// and filenames
      /// \return True if request received.
      private: bool EntitySystemInfoService(const msgs::Empty &_req,
                                            msgs::EntityPlugin_V &_res);

      /// \brief All the systems.
      private: std::vector<SystemInternal> systems;

      /// \brief Pending systems to be added to systems.
      private: std::vector<SystemInternal> pendingSystems;

      /// \brief Mutex to protect pendingSystems
      private: mutable std::mutex pendingSystemsMutex;

      /// \brief Systems implementing Configure
      private: std::vector<ISystemConfigure *> systemsConfigure;

      /// \brief Systems implementing ConfigureParameters
      private: std::vector<ISystemConfigureParameters *>
        systemsConfigureParameters;

      /// \brief Systems implementing Reset
      private: std::vector<ISystemReset *> systemsReset;

      /// \brief Systems implementing PreUpdate
      private: PrioritizedSystems<ISystemPreUpdate *> systemsPreupdate;

      /// \brief Systems implementing Update
      private: PrioritizedSystems<ISystemUpdate *> systemsUpdate;

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

      /// \brief A list of entity systems to add
      private: std::vector<msgs::EntityPlugin_V> systemsToAdd;

      /// \brief Mutex to protect systemsToAdd list
      private: std::mutex systemsMsgMutex;

      /// \brief Node for communication.
      private: std::unique_ptr<transport::Node> node{nullptr};

      /// \brief Pointer to associated parameters registry
      private: gz::transport::parameters::ParametersRegistry *
        parametersRegistry;
    };
    }
  }  // namespace sim
}  // namespace gz
#endif  // GZ_SIM_SYSTEMINTERNAL_HH_
