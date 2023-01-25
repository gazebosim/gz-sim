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

#include <list>
#include <set>

#include <gz/common/StringUtils.hh>

#include "gz/sim/components/SystemPluginInfo.hh"
#include "gz/sim/Conversions.hh"
#include "SystemManager.hh"

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
SystemManager::SystemManager(
  const SystemLoaderPtr &_systemLoader,
  EntityComponentManager *_entityCompMgr,
  EventManager *_eventMgr,
  const std::string &_namespace,
  gz::transport::parameters::ParametersRegistry *_parametersRegistry)
  : systemLoader(_systemLoader),
    entityCompMgr(_entityCompMgr),
    eventMgr(_eventMgr),
    parametersRegistry(_parametersRegistry)
{
  transport::NodeOptions opts;
  opts.SetNameSpace(_namespace);
  this->node = std::make_unique<transport::Node>(opts);
  std::string entitySystemAddService{"entity/system/add"};
  this->node->Advertise(entitySystemAddService,
      &SystemManager::EntitySystemAddService, this);
  gzmsg << "Serving entity system service on ["
         << "/" << entitySystemAddService << "]" << std::endl;

  std::string entitySystemInfoService{"system/info"};
  this->node->Advertise(entitySystemInfoService,
      &SystemManager::EntitySystemInfoService, this);
}

//////////////////////////////////////////////////
void SystemManager::LoadPlugin(const Entity _entity,
                               const sdf::Plugin &_plugin)
{
  std::optional<SystemPluginPtr> system;
  {
    std::lock_guard<std::mutex> lock(this->systemLoaderMutex);
    system = this->systemLoader->LoadPlugin(_plugin);
  }

  // System correctly loaded from library
  if (system)
  {
    SystemInternal ss(system.value(), _entity);
    ss.fname = _plugin.Filename();
    ss.name = _plugin.Name();
    ss.configureSdf = _plugin.ToElement();
    this->AddSystemImpl(ss, ss.configureSdf);
    gzdbg << "Loaded system [" << _plugin.Name()
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

    if (system.configureParameters)
      this->systemsConfigureParameters.push_back(system.configureParameters);

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
  /// \brief Filename of the plugin library
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

  for (auto &system : this->systems)
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
        gzwarn << "In-memory without ISystemReset detected: ["
          << system.name << "]\n"
          << "Systems created without plugins that do not implement Reset"
          << " will not be reloaded. Reset may not work correctly\n";
        continue;
      }

      sdf::ElementPtr elem = system.configureSdf ?
          system.configureSdf->Clone() : nullptr;
      PluginInfo info = {system.parentEntity, system.fname, system.name, elem};

      pluginsToBeLoaded.push_back(info);
    }
  }

  this->systems.clear();

  // Load plugins which do not implement reset after clearing this->systems
  // to ensure the previous instance is destroyed before the new one is created
  // and configured.
  for (const auto &pluginInfo : pluginsToBeLoaded)
  {
    sdf::Plugin plugin;
    plugin.Load(pluginInfo.sdf);
    plugin.SetFilename(pluginInfo.fname);
    plugin.SetName(pluginInfo.name);
    this->LoadPlugin(pluginInfo.entity, plugin);
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
  // Add component
  if (this->entityCompMgr && kNullEntity != _system.parentEntity)
  {
    msgs::Plugin_V systemInfoMsg;
    auto systemInfoComp =
        this->entityCompMgr->Component<components::SystemPluginInfo>(
        _system.parentEntity);
    if (systemInfoComp)
    {
      systemInfoMsg = systemInfoComp->Data();
    }
    if (_sdf)
    {
      auto pluginMsg = systemInfoMsg.add_plugins();
      pluginMsg->CopyFrom(convert<msgs::Plugin>(*_sdf.get()));
    }

    this->entityCompMgr->SetComponentData<components::SystemPluginInfo>(
        _system.parentEntity, systemInfoMsg);
    this->entityCompMgr->SetChanged(_system.parentEntity,
        components::SystemPluginInfo::typeId);
  }

  // Configure the system, if necessary
  if (_system.configure && this->entityCompMgr && this->eventMgr)
  {
    _system.configure->Configure(_system.parentEntity, _sdf,
                                 *this->entityCompMgr,
                                 *this->eventMgr);
  }

  // Configure the system parameters, if necessary
  if (
    _system.configureParameters && this->entityCompMgr &&
    this->parametersRegistry)
  {
    _system.configureParameters->ConfigureParameters(
      *this->parametersRegistry,
      *this->entityCompMgr);
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

const std::vector<ISystemConfigureParameters *>&
SystemManager::SystemsConfigureParameters()
{
  return this->systemsConfigureParameters;
}

//////////////////////////////////////////////////
const std::vector<ISystemReset *> &SystemManager::SystemsReset()
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
  auto checkEntity = [&](const SystemInternal &_system)
      {
        return _system.parentEntity == _entity;
      };

  std::vector<SystemInternal> result;
  std::copy_if(this->systems.begin(), this->systems.end(),
      std::back_inserter(result), checkEntity);
  std::copy_if(this->pendingSystems.begin(), this->pendingSystems.end(),
      std::back_inserter(result), checkEntity);
  return result;
}

//////////////////////////////////////////////////
bool SystemManager::EntitySystemAddService(const msgs::EntityPlugin_V &_req,
                                           msgs::Boolean &_res)
{
  std::lock_guard<std::mutex> lock(this->systemsMsgMutex);
  this->systemsToAdd.push_back(_req);
  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool SystemManager::EntitySystemInfoService(const msgs::Empty &,
                                            msgs::EntityPlugin_V &_res)
{
  // loop through all files in paths and populate the list of
  // plugin libraries.
  std::list<std::string> paths = this->systemLoader->PluginPaths();
  std::set<std::string> filenames;
  for (const auto &p : paths)
  {
    if (common::exists(p))
    {
      for (common::DirIter file(p);
          file != common::DirIter(); ++file)
      {
        std::string current(*file);
        std::string filename = common::basename(current);
        if (common::isFile(current) &&
            (common::EndsWith(filename, ".so") ||
             common::EndsWith(filename, ".dll") ||
             common::EndsWith(filename, ".dylib")))
        {
          // remove extension and lib prefix
          size_t extensionIndex = filename.rfind(".");
          std::string nameWithoutExtension =
              filename.substr(0, extensionIndex);
          if (common::StartsWith(nameWithoutExtension, "lib"))
          {
            nameWithoutExtension = nameWithoutExtension.substr(3);
          }
          filenames.insert(nameWithoutExtension);
        }
      }
    }
  }

  for (const auto &fn : filenames)
  {
    auto plugin = _res.add_plugins();
    plugin->set_filename(fn);
  }

  return true;
}

//////////////////////////////////////////////////
void SystemManager::ProcessPendingEntitySystems()
{
  std::lock_guard<std::mutex> lock(this->systemsMsgMutex);
  for (auto &req : this->systemsToAdd)
  {
    Entity entity = req.entity().id();

    if (req.plugins().empty())
    {
      gzwarn << "Unable to add plugins to Entity: '" << entity
             << "'. No plugins specified." << std::endl;
       continue;
    }

    for (auto &pluginMsg : req.plugins())
    {
      std::string fname = pluginMsg.filename();
      std::string name = pluginMsg.name();
      std::string innerxml = pluginMsg.innerxml();
      sdf::Plugin pluginSDF(fname, name, innerxml);
      this->LoadPlugin(entity, pluginSDF);
    }
  }
  this->systemsToAdd.clear();
}
