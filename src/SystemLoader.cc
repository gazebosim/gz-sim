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

#include <optional>
#include <string>
#include <unordered_set>

#include <ignition/gazebo/SystemLoader.hh>

#include <sdf/Element.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Util.hh>

#include <ignition/plugin/Loader.hh>

#include <ignition/gazebo/config.hh>

using namespace ignition::gazebo;

class ignition::gazebo::SystemLoaderPrivate
{
  //////////////////////////////////////////////////
  public: explicit SystemLoaderPrivate() = default;

  //////////////////////////////////////////////////
  public: bool InstantiateSystemPlugin(const std::string &_filename,
              const std::string &_name,
              const sdf::ElementPtr &/*_sdf*/,
              ignition::plugin::PluginPtr &_plugin)
  {
    ignition::common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv(pluginPathEnv);

    for (const auto &path : this->systemPluginPaths)
      systemPaths.AddPluginPaths(path);

    std::string homePath;
    ignition::common::env(IGN_HOMEDIR, homePath);
    systemPaths.AddPluginPaths(homePath + "/.ignition/gazebo/plugins");
    systemPaths.AddPluginPaths(IGN_GAZEBO_PLUGIN_INSTALL_DIR);

    auto pathToLib = systemPaths.FindSharedLibrary(_filename);
    if (pathToLib.empty())
    {
      // We assume ignition::gazebo corresponds to the levels feature
      if (_name != "ignition::gazebo")
      {
        ignerr << "Failed to load system plugin [" << _filename <<
                  "] : couldn't find shared library." << std::endl;
      }
      return false;
    }

    auto pluginNames = this->loader.LoadLib(pathToLib);
    if (pluginNames.empty())
    {
      ignerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't load library on path [" << pathToLib <<
                "]." << std::endl;
      return false;
    }

    std::string pluginName = *pluginNames.begin();
    for (auto name : pluginNames)
    {
      if (name.find("gazebo") != std::string::npos)
        pluginName = name;
    }

    if (pluginName.empty())
    {
      ignerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't load library on path [" << pathToLib <<
                "]." << std::endl;
      return false;
    }

    _plugin = this->loader.Instantiate(_name);
    if (!_plugin)
    {
      ignerr << "Failed to load system plugin [" << _name <<
                "] : could not instantiate from library [" << _filename <<
                "] from path [" << pathToLib << "]." << std::endl;
      return false;
    }

    if (!_plugin->HasInterface<System>())
    {
      ignerr << "Failed to load system plugin [" << _name <<
        "] : system not found in library  [" << _filename <<
        "] from path [" << pathToLib << "]." << std::endl;

      return false;
    }

    this->systemPluginsAdded.insert(_plugin);
    return true;
  }

  // Default plugin search path environment variable
  public: std::string pluginPathEnv{"IGN_GAZEBO_SYSTEM_PLUGIN_PATH"};

  /// \brief Plugin loader instace
  public: ignition::plugin::Loader loader;

  /// \brief Paths to search for system plugins.
  public: std::unordered_set<std::string> systemPluginPaths;

  /// \brief System plugins that have instances loaded via the manager.
  public: std::unordered_set<SystemPluginPtr> systemPluginsAdded;
};

//////////////////////////////////////////////////
SystemLoader::SystemLoader()
  : dataPtr(new SystemLoaderPrivate())
{
}

//////////////////////////////////////////////////
SystemLoader::~SystemLoader() = default;

//////////////////////////////////////////////////
void SystemLoader::AddSystemPluginPath(const std::string &_path)
{
  this->dataPtr->systemPluginPaths.insert(_path);
}

//////////////////////////////////////////////////
std::optional<SystemPluginPtr> SystemLoader::LoadPlugin(
  const std::string &_filename,
  const std::string &_name,
  const sdf::ElementPtr &_sdf)
{
  ignition::plugin::PluginPtr plugin;

  if (_filename == "" || _name == "")
  {
    ignerr << "Failed to instantiate system plugin: empty argument "
              "[(filename): " << _filename << "] " <<
              "[(name): " << _name << "]." << std::endl;
    return {};
  }

  auto ret = this->dataPtr->InstantiateSystemPlugin(_filename,
                                                    _name,
                                                    _sdf, plugin);
  if (ret && plugin)
  {
    return plugin;
  }

  return {};
}

//////////////////////////////////////////////////
std::optional<SystemPluginPtr> SystemLoader::LoadPlugin(
  const sdf::ElementPtr &_sdf)
{
  if (nullptr == _sdf)
  {
    return {};
  }
  auto filename = _sdf->Get<std::string>("filename");
  auto pluginName = _sdf->Get<std::string>("name");
  return LoadPlugin(filename, pluginName, _sdf);
}

//////////////////////////////////////////////////
std::string SystemLoader::PrettyStr() const
{
  return this->dataPtr->loader.PrettyStr();
}

