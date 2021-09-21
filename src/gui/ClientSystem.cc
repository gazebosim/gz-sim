/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "ClientSystem.hh"

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/fuel_tools/Interface.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
class ignition::gazebo::ClientSystem::Implementation
{
  /// \brief World name
  public: std::string worldName;

  public: EventManager* eventManager;

  public: std::set<std::string> pendingPlugins;

  public: std::mutex pluginMutex;

  public: std::chrono::milliseconds updatePeriod {1000/60};

  /// \brief Keep track of the last time the update has been run. Only applies
  /// to same process.
  public: std::chrono::time_point<std::chrono::system_clock>
      lastSameProcessUpdate{std::chrono::system_clock::now()};

};

//////////////////////////////////////////////////
ClientSystem::ClientSystem()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{

}

//////////////////////////////////////////////////
ClientSystem::~ClientSystem() = default;

//////////////////////////////////////////////////
void ClientSystem::WorldName(const std::string &_worldName)
{
  this->dataPtr->worldName = _worldName;
}

//////////////////////////////////////////////////
std::string ClientSystem::WorldName() const
{
  return this->dataPtr->worldName;
}

//////////////////////////////////////////////////
void ClientSystem::Configure(const Entity &/*_entity*/,
  const std::shared_ptr<const sdf::Element> &/*_sdf*/,
  EntityComponentManager &/*_ecm*/,
  EventManager &_eventMgr)
{
  igndbg << "ClientSystem::Configure" << std::endl;

  this->dataPtr->eventManager = &_eventMgr;

  auto win = gui::App()->findChild<ignition::gui::MainWindow *>();
  auto winWorldNames = win->property("worldNames").toStringList();
  winWorldNames.append(QString::fromStdString(this->dataPtr->worldName));
  win->setProperty("worldNames", winWorldNames);
}


//////////////////////////////////////////////////
void ClientSystem::OnPluginAdded(const std::string &_objectName)
{
  igndbg << "ClientSystem::OnPluginAdded" << std::endl;
  std::lock_guard<std::mutex> lock(this->dataPtr->pluginMutex);
  this->dataPtr->pendingPlugins.insert(_objectName);
}

//////////////////////////////////////////////////
void ClientSystem::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  // Process pending plugins first
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->pluginMutex);
    for(auto pluginName: this->dataPtr->pendingPlugins)
    {
      auto plugin = gui::App()->PluginByName(pluginName);
      if (!plugin)
      {
        ignerr << "Failed to get plugin [" << pluginName 
               << "]" << std::endl;
        return;
      }
      auto guiSystem = dynamic_cast<GuiSystem *>(plugin.get());

      if (guiSystem) {
        guiSystem->Configure(*this->dataPtr->eventManager, true);
      }
    }
    this->dataPtr->pendingPlugins.clear();
  }

  bool changeEvent = _ecm.HasEntitiesMarkedForRemoval() ||
    _ecm.HasNewEntities() || _ecm.HasOneTimeComponentChanges();
  auto now = std::chrono::system_clock::now();
  bool itsTime =  now - this->dataPtr->lastSameProcessUpdate > this->dataPtr->updatePeriod;

  if (changeEvent || itsTime)
  {
    igndbg << "ClientSystem::PreUpdate Exec" << std::endl;
    auto plugins = gui::App()->findChildren<GuiSystem *>();
    for (auto plugin : plugins)
    {
      plugin->Update(_info, _ecm);
    }
  }
  else
  {
    igndbg << "ClientSystem::PreUpdate Skip" << std::endl;
  }
}


//////////////////////////////////////////////////
void ClientSystem::PostUpdate(const UpdateInfo &_info,
  const EntityComponentManager &_ecm)
{
  bool changeEvent = _ecm.HasEntitiesMarkedForRemoval() ||
    _ecm.HasNewEntities() || _ecm.HasOneTimeComponentChanges();
  auto now = std::chrono::system_clock::now();
  bool itsTime =  now - this->dataPtr->lastSameProcessUpdate > this->dataPtr->updatePeriod;

  igndbg << "ClientSystem::PostUpdate: " << _info.iterations << std::endl;

  if (changeEvent || itsTime)
  {
    igndbg << "ClientSystem::PostUpdate Exec" << std::endl;
    auto plugins = gui::App()->findChildren<GuiSystem *>();
    for (auto plugin : plugins)
    {
      plugin->PostUpdate(_info, _ecm);
    }

    this->dataPtr->lastSameProcessUpdate = now;
  }
  else
  {
    igndbg << "ClientSystem::PostUpdate Skip" << std::endl;
  }
}
