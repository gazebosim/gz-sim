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

#include "SystemManager.hh"

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
SystemManager::SystemManager(const SystemLoaderPtr &_systemLoader,
                             EntityComponentManager *_entityCompMgr,
                             EventManager *_eventMgr)
  : systemLoader(_systemLoader),
    entityCompMgr(_entityCompMgr),
    eventMgr(_eventMgr)
{
}

//////////////////////////////////////////////////
void SystemManager::LoadPlugin(const Entity _entity,
                               const std::string &_fname,
                               const std::string &_name,
                               const sdf::ElementPtr &_sdf)
{
  std::optional<SystemPluginPtr> system;
  {
    std::lock_guard<std::mutex> lock(this->systemLoaderMutex);
    system = this->systemLoader->LoadPlugin(_fname, _name, _sdf);
  }

  // System correctly loaded from library
  if (system)
  {
    SystemInternal ss(system.value(), _entity);
    ss.fname = _fname;
    ss.name = _name;
    ss.configureSdf = _sdf;
    this->AddSystemImpl(ss, ss.configureSdf);
    gzdbg << "Loaded system [" << _name
           << "] for entity [" << _entity << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
size_t SystemManager::TotalCount() const
{
  return this->ActiveCount() + this->PendingCount();
}

//////////////////////////////////////////////////
size_t SystemManager::ActiveCount() const
{
  return this->systems.size();
}

//////////////////////////////////////////////////
size_t SystemManager::PendingCount() const
{
  std::lock_guard<std::mutex> lock(this->pendingSystemsMutex);
  return this->pendingSystems.size();
}

//////////////////////////////////////////////////
size_t SystemManager::ActivatePendingSystems()
{
  std::lock_guard<std::mutex> lock(this->pendingSystemsMutex);

  auto count = this->pendingSystems.size();

  for (const auto& system : this->pendingSystems)
  {
    this->systems.push_back(system);

    if (system.configure)
      this->systemsConfigure.push_back(system.configure);

    if (system.reset)
      this->systemsReset.push_back(system.reset);

    if (system.preupdate)
      this->systemsPreupdate.push_back(system.preupdate);

    if (system.update)
      this->systemsUpdate.push_back(system.update);

    if (system.postupdate)
      this->systemsPostupdate.push_back(system.postupdate);
  }

  this->pendingSystems.clear();
  return count;
}

//////////////////////////////////////////////////
/// \brief Structure to temporarily store plugin information for reset
struct PluginInfo {
  /// \brief Entity plugin is attached to
  Entity entity;
  /// \brief  Filename of the plugin library
  std::string fname;
  /// \brief Name of the plugin
  std::string name;
  /// \brief SDF element (content of the plugin tag)
  sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
void SystemManager::Reset(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  {
    std::lock_guard<std::mutex> lock(this->pendingSystemsMutex);
    this->pendingSystems.clear();
  }

  // Clear all iterable collections of systems
  this->systemsConfigure.clear();
  this->systemsReset.clear();
  this->systemsPreupdate.clear();
  this->systemsUpdate.clear();
  this->systemsPostupdate.clear();

  std::vector<PluginInfo> pluginsToBeLoaded;

  for (auto& system : this->systems)
  {
    if (nullptr != system.reset)
    {
      // If implemented, call reset and add to pending systems.
      system.reset->Reset(_info, _ecm);

      {
        std::lock_guard<std::mutex> lock(this->pendingSystemsMutex);
        this->pendingSystems.push_back(system);
      }
    }
    else
    {
      // Cannot reset systems that were created in memory rather than
      // from a plugin, because there isn't access to the constructor.
      if (nullptr != system.systemShared)
      {
        ignwarn << "Systems not created from plugins cannot be correctly "
          << " reset without implementing ISystemReset interface.\n";
          continue;
      }


      PluginInfo info = {
        system.parentEntity, system.fname, system.name,
        system.configureSdf->Clone()
      };

      pluginsToBeLoaded.push_back(info);
    }
  }

  this->systems.clear();

  // Load plugins which do not implement reset after clearing this->systems
  // to ensure the previous instance is destroyed before the new one is created
  // and configured.
  for (const auto &pluginInfo : pluginsToBeLoaded) {
    this->LoadPlugin(pluginInfo.entity, pluginInfo.fname, pluginInfo.name,
        pluginInfo.sdf);
  }
  this->ActivatePendingSystems();
}

//////////////////////////////////////////////////
void SystemManager::AddSystem(const SystemPluginPtr &_system,
      Entity _entity,
      std::shared_ptr<const sdf::Element> _sdf)
{
  this->AddSystemImpl(SystemInternal(_system, _entity), _sdf);
}

//////////////////////////////////////////////////
void SystemManager::AddSystem(
      const std::shared_ptr<System> &_system,
      Entity _entity,
      std::shared_ptr<const sdf::Element> _sdf)
{
  this->AddSystemImpl(SystemInternal(_system, _entity), _sdf);
}

//////////////////////////////////////////////////
void SystemManager::AddSystemImpl(
      SystemInternal _system,
      std::shared_ptr<const sdf::Element> _sdf)
{
  // Configure the system, if necessary
  if (_system.configure && this->entityCompMgr && this->eventMgr)
  {
    _system.configure->Configure(_system.parentEntity, _sdf,
                                 *this->entityCompMgr,
                                 *this->eventMgr);
  }

  // Update callbacks will be handled later, add to queue
  std::lock_guard<std::mutex> lock(this->pendingSystemsMutex);
  this->pendingSystems.push_back(_system);
}

//////////////////////////////////////////////////
const std::vector<ISystemConfigure *>& SystemManager::SystemsConfigure()
{
  return this->systemsConfigure;
}

//////////////////////////////////////////////////
const std::vector<ISystemReset *>& SystemManager::SystemsReset()
{
  return this->systemsReset;
}

//////////////////////////////////////////////////
const std::vector<ISystemPreUpdate *>& SystemManager::SystemsPreUpdate()
{
  return this->systemsPreupdate;
}

//////////////////////////////////////////////////
const std::vector<ISystemUpdate *>& SystemManager::SystemsUpdate()
{
  return this->systemsUpdate;
}

//////////////////////////////////////////////////
const std::vector<ISystemPostUpdate *>& SystemManager::SystemsPostUpdate()
{
  return this->systemsPostupdate;
}

//////////////////////////////////////////////////
std::vector<SystemInternal> SystemManager::TotalByEntity(Entity _entity)
{
  std::vector<SystemInternal> result;
  for (auto system : this->systems)
  {
    if (system.parentEntity == _entity)
      result.push_back(system);
  }
  for (auto system : this->pendingSystems)
  {
    if (system.parentEntity == _entity)
      result.push_back(system);
  }
  return result;
}
