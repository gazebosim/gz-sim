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

#include <ignition/gazebo/config.hh>

using namespace ignition::gazebo;
using SystemPtr = SystemManager::SystemPtr;

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
  struct SystemInfo
  {
    /// \brief Alias that this system is known as
    std::string alias;

    /// \brief Fully namespaced class name to be instanced
    std::string classname;

    /// \brief Shared library that this class is instanced from
    std::string filename;

    /// \brief Fully resolved path to the library
    std::string path;
  };

  //////////////////////////////////////////////////
  public: explicit SystemManagerPrivate():
          defaultConfigPath(ignition::common::joinPaths(homePath(), ".ignition",
                            "gazebo", "default.config"))
  {
  }

  //////////////////////////////////////////////////
  public: bool InstantiateSystemPlugin(const std::string &_alias,
                                       ignition::plugin::PluginPtr &_plugin)
  {
    const auto &it = this->knownSystems.find(_alias);
    if (it == this->knownSystems.end())
    {
      ignerr << "Failed to load system plugin: "  <<
                "unknown alias [" << _alias << "]." << std::endl;
      return false;
    }

    const auto &info = it->second;

    auto pluginNames = loader.LoadLibrary(info.path);
    if (pluginNames.empty())
    {
      ignerr << "Failed to load system plugin [" << info.filename <<
                "] : couldn't load library on path [" << info.path <<
                "]." << std::endl;
      return false;
    }

    auto pluginName = *pluginNames.begin();
    if (pluginName.empty())
    {
      ignerr << "Failed to load system plugin [" << info.filename <<
                "] : couldn't load library on path [" << info.path <<
                "]." << std::endl;
      return false;
    }

    auto validPlugins = loader.PluginsImplementing<System>();
    if (validPlugins.count(info.classname) == 0) {
      ignerr << "Failed to load system plugin [" << info.classname <<
                "] : system not found in library  [" << info.filename <<
                "] from path [" << info.path << "]." << std::endl;
      return false;
    }

    _plugin = loader.Instantiate(info.classname);
    if (!_plugin)
    {
      ignerr << "Failed to load system plugin [" << info.filename <<
                "] : couldn't instantiate plugin on path [" << info.path <<
                "]." << std::endl;
      return false;
    }

    systemPluginsAdded.insert(_plugin);
    return true;
  }

  //////////////////////////////////////////////////
  public: bool AddSystemPlugin(const std::string &_alias,
                               const std::string &_filename,
                               const std::string &_classname)
  {
    ignition::common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv(pluginPathEnv);

    for (const auto &path : systemPluginPaths)
      systemPaths.AddPluginPaths(path);

    auto home = homePath();
    systemPaths.AddPluginPaths(home + "/.ignition/gazebo/plugins");
    systemPaths.AddPluginPaths(IGN_GAZEBO_PLUGIN_INSTALL_DIR);

    auto pathToLib = systemPaths.FindSharedLibrary(_filename);
    if (pathToLib.empty())
    {
      ignerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't find shared library." << std::endl;
      return false;
    }

    SystemInfo info;
    info.alias = _alias;
    info.filename = _filename;
    info.classname = _classname;
    info.path = pathToLib;

    if (knownSystems.find(_alias) != knownSystems.end())
    {
      ignerr << "Failed to load system plugin [" << _filename <<
                "] : duplicate system alias [" << _alias << "]" << std::endl;
      return false;
    }

    knownSystems[_alias] = info;
    return true;
  }

  // Default plugin search path environment variable
  public: std::string pluginPathEnv{"IGN_GAZEBO_SYSTEM_PLUGIN_PATH"};

  /// \brief Location of the default system plugin configuration
  public: std::string defaultConfigPath;

  /// \brief Plugin loader instace
  public: ignition::plugin::Loader loader;

  /// \brief Paths to search for system plugins.
  public: std::unordered_set<std::string> systemPluginPaths;

  /// \brief System plugins that have instances loaded via the manager.
  public: std::unordered_set<ignition::plugin::PluginPtr> systemPluginsAdded;

  /// \brief Information about known system plugins loaded from config.
  public: std::map<std::string, SystemInfo> knownSystems;
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

//////////////////////////////////////////////////
void SystemManager::AddSystemPluginPath(const std::string &_path)
{
  this->dataPtr->systemPluginPaths.insert(_path);
}

//////////////////////////////////////////////////
bool SystemManager::LoadSystemConfig(const std::string &_config)
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
    auto alias = systemElem->Attribute("alias");
    auto filename = systemElem->Attribute("filename");
    auto classname = systemElem->Attribute("classname");

    this->dataPtr->AddSystemPlugin(alias, filename, classname);
  }

  return true;
}

//////////////////////////////////////////////////
SystemPtr SystemManager::Instantiate(const std::string &_alias)
{
  ignition::plugin::PluginPtr plugin;
  this->dataPtr->InstantiateSystemPlugin(_alias, plugin);

  if (plugin)
  {
    return plugin->QueryInterfaceSharedPtr<System>();
  }
  else
    return nullptr;
  {
  }
}

//////////////////////////////////////////////////
std::string SystemManager::PrettyStr() const
{
  return this->dataPtr->loader.PrettyStr();
}

