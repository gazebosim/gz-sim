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
#include "ignition/gazebo/SystemManager.hh"

#include <tinyxml2.h>

#include <string>
#include <unordered_set>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Util.hh>

#include <ignition/plugin/Loader.hh>

using namespace ignition::gazebo;

std::string homePath()
{
  std::string homePath;
#ifndef _WIN32
  ignition::common::env("HOME", homePath);
#else
  ignition::common::env("HOMEPATH", homePath);
#endif

  return homePath;
}

class ignition::gazebo::SystemManagerPrivate
{
  public: explicit SystemManagerPrivate():
          defaultConfigPath(ignition::common::joinPaths(homePath(), ".ignition",
                            "gazebo", "default.config"))
  {
  }

  public: bool instantiateSystemPlugin(const std::string &_pathToLib,
                                       const std::string &_filename,
                                       const std::string &_name,
                                       ignition::plugin::PluginPtr &_plugin)
  {
    auto pluginNames = loader.LoadLibrary(_pathToLib);
    if (pluginNames.empty())
    {
      ignerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't load library on path [" << _pathToLib <<
                "]." << std::endl;
      return false;
    }

    auto pluginName = *pluginNames.begin();
    if (pluginName.empty())
    {
      ignerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't load library on path [" << _pathToLib <<
                "]." << std::endl;
      return false;
    }

    auto validPlugins = loader.PluginsImplementing<System>();
    if (validPlugins.count(_name) == 0) {
      ignerr << "Failed to load system plugin [" << _name <<
                "] : system not found in library  [" << _filename <<
                "] from path [" << _pathToLib << "]." << std::endl;
      return false;
    }

    _plugin = loader.Instantiate(_name);
    if (!_plugin)
    {
      ignerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't instantiate plugin on path [" << _pathToLib <<
                "]." << std::endl;
      return false;
    }

    return true;
  }

  public: bool loadSystemPlugin(const std::string &_filename,
                                const std::string &_name,
                                const tinyxml2::XMLElement *_pluginElem)
  {
    (void) _pluginElem;
    ignmsg << "Loading system [" << _name << "]"  <<
              " from [" << _filename << "]" << std::endl;

    // Get full path
    auto home = homePath();

    ignition::common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv(pluginPathEnv);

    for (const auto &path : systemPluginPaths)
      systemPaths.AddPluginPaths(path);

    std::cout << "paths checked" << std::endl;
    for (const auto &path : systemPaths.PluginPaths())
      std::cout << path << std::endl;

    auto pathToLib = systemPaths.FindSharedLibrary(_filename);
    if (pathToLib.empty())
    {
      ignerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't find shared library." << std::endl;
      return false;
    }

    ignition::plugin::PluginPtr pluginPtr;
    if (!instantiateSystemPlugin(pathToLib, _filename, _name, pluginPtr))
    {
      return false;
    }

    auto plugin = pluginPtr->QueryInterfaceSharedPtr<System>();
    if (!plugin)
    {
      ignerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't get interface [ignition::gazebo::System]." <<
                std::endl;
      return false;
    }

    // Store plugin in queue to be added to the window
    systemPluginsAdded.push_back(plugin);

    return true;
  }

  public: std::string pluginPathEnv = "IGN_GAZEBO_SYSTEM_PLUGIN_PATH";
  public: std::string defaultConfigPath;
  public: std::unordered_set<std::string> systemPluginPaths;
  public: std::vector<std::shared_ptr<System>> systemPluginsAdded;
  public: ignition::plugin::Loader loader;
};

//////////////////////////////////////////////////
SystemManager::SystemManager()
  : dataPtr(new SystemManagerPrivate())
{
}

//////////////////////////////////////////////////
SystemManager::~SystemManager()
{

}

const std::vector<std::shared_ptr<System>>& SystemManager::GetLoadedSystems() const
{
  return this->dataPtr->systemPluginsAdded;
}

void SystemManager::addSystemPluginPath(const std::string &_path)
{
  this->dataPtr->systemPluginPaths.insert(_path);
}

bool SystemManager::loadSystemConfig(const std::string& _config)
{
  if (_config.empty())
  {
    ignerr << "Missing config file" << std::endl;
    return false;
  }

  tinyxml2::XMLDocument doc;
  auto success = !doc.LoadFile(_config.c_str());

  if (!success)
  {
    if (_config != this->dataPtr->defaultConfigPath)
    {
      ignerr << "Failed to load file [" << _config << "]: XMLError"
             << std::endl;
    }

    return false;
  }

  ignmsg << "Loading config [" << _config << "]" << std::endl;

  // Clear all previously added system plugins
  this->dataPtr->systemPluginsAdded.clear();

  for (auto systemElem = doc.FirstChildElement("system"); systemElem != nullptr;
      systemElem = systemElem->NextSiblingElement("system"))
  {
    auto filename = systemElem->Attribute("filename");
    auto name = systemElem->Attribute("name");
    this->dataPtr->loadSystemPlugin(filename, name, systemElem);
  }

  return true;
}

std::string SystemManager::PrettyStr() const
{
  return this->dataPtr->loader.PrettyStr();
}

