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

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
SystemManager::SystemManager(const SystemLoaderPtr &_systemLoader)
  : systemLoader(_systemLoader)
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
    this->AddSystem(system.value(), _entity, _sdf);
    igndbg << "Loaded system [" << _name
           << "] for entity [" << _entity << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
size_t SystemManager::TotalCount() const
{
  std::lock_guard<std::mutex> lock(this->systemsMutex);
  return this->systems.size() + this->pendingSystems.size();
}

//////////////////////////////////////////////////
size_t SystemManager::ActiveCount() const
{
  std::lock_guard<std::mutex> lock(this->systemsMutex);
  return this->systems.size();
}

//////////////////////////////////////////////////
size_t SystemManager::PendingCount() const
{
  std::lock_guard<std::mutex> lock(this->systemsMutex);
  return this->pendingSystems.size();
}

//////////////////////////////////////////////////
void SystemManager::ConfigurePendingSystems(EntityComponentManager &_ecm,
                                            EventManager &_eventMgr)
{
  std::lock_guard<std::mutex> lock(this->systemsMutex);
  for (size_t ii = 0; ii < this->pendingSystems.size(); ++ii)
  {
    if (this->pendingSystemsConfigured[ii])
      continue;

    const auto& system = this->pendingSystems[ii];

    if (system.configure)
    {
      system.configure->Configure(system.configureEntity,
                                  system.configureSdf,
                                  _ecm, _eventMgr);
      this->pendingSystemsConfigured[ii] = true;
    }
  }
}

//////////////////////////////////////////////////
size_t SystemManager::ActivatePendingSystems()
{
  std::lock_guard<std::mutex> lock(this->systemsMutex);

  auto count = this->pendingSystems.size();

  for (const auto& system : this->pendingSystems)
  {
    this->systems.push_back(system);

    if (system.configure)
      this->systemsConfigure.push_back(system.configure);

    if (system.preupdate)
      this->systemsPreupdate.push_back(system.preupdate);

    if (system.update)
      this->systemsUpdate.push_back(system.update);

    if (system.postupdate)
      this->systemsPostupdate.push_back(system.postupdate);
  }

  this->pendingSystems.clear();
  this->pendingSystemsConfigured.clear();
  return count;
}

//////////////////////////////////////////////////
void SystemManager::AddSystem(const SystemPluginPtr &_system,
      Entity _entity,
      std::shared_ptr<const sdf::Element> _sdf)
{
  this->AddSystemImpl(SystemInternal(_system), _entity, _sdf);
}

//////////////////////////////////////////////////
void SystemManager::AddSystem(
      const std::shared_ptr<System> &_system,
      Entity _entity,
      std::shared_ptr<const sdf::Element> _sdf)
{
  this->AddSystemImpl(SystemInternal(_system), _entity, _sdf);
}

//////////////////////////////////////////////////
void SystemManager::AddSystemImpl(
      SystemInternal _system,
      Entity _entity,
      std::shared_ptr<const sdf::Element> _sdf)
{
  _system.configureEntity = _entity;
  _system.configureSdf = _sdf;

  // Update callbacks will be handled later, add to queue
  std::lock_guard<std::mutex> lock(this->systemsMutex);
  this->pendingSystems.push_back(_system);
  this->pendingSystemsConfigured.push_back(false);
}

//////////////////////////////////////////////////
const std::vector<ISystemConfigure *>& SystemManager::SystemsConfigure()
{
  return this->systemsConfigure;
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
