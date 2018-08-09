/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <unordered_set>

#include "ignition/gazebo/SystemPluginManager.hh"

#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/PluginPtr.hh>


using namespace ignition::gazebo;

class ignition::gazebo::SystemPluginManagerPrivate
{
  public: explicit SystemPluginManagerPrivate()
  {
  }

  std::map<std::string, std::unordered_set<std::string>> loaded_plugins;

  public: ignition::plugin::Loader loader;
};

//////////////////////////////////////////////////
SystemPluginManager::SystemPluginManager()
  : dataPtr(new SystemPluginManagerPrivate())
{

}

//////////////////////////////////////////////////
SystemPluginManager::~SystemPluginManager()
{

}

void
SystemPluginManager::LoadLibrary(const std::string &_pathToLibrary) {
  auto plugins = this->dataPtr->loader.LoadLibrary(_pathToLibrary);

  this->dataPtr->loaded_plugins[_pathToLibrary] = plugins;
}

using SystemPluginPtr = ignition::plugin::SpecializedPluginPtr<ignition::gazebo::System>;

SystemPluginPtr
SystemPluginManager::Instantiate(const std::string &_pluginName) const {
  return this->dataPtr->loader.Instantiate(_pluginName);
}

std::unordered_set<std::string>
SystemPluginManager::PluginsByType(const SystemTypeId& _system_type) {

  std::unordered_set<std::string> plugins;

  for(auto it: this->dataPtr->loaded_plugins) {
    for(auto plugin_name: it.second) {
      auto plugin = this->Instantiate(plugin_name);
      if(_system_type == plugin->QueryInterface<ignition::gazebo::System>()->SystemType()) {
        plugins.insert(plugin_name);
      }
    }
  }
  return plugins;
}

