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

#include <gz/sim/SystemLoader.hh>

#include <sdf/Element.hh>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/StringUtils.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/Util.hh>

#include <gz/plugin/Loader.hh>

#include <gz/sim/config.hh>

using namespace gz::sim;

class ignition::gazebo::SystemLoaderPrivate
{
  //////////////////////////////////////////////////
  public: explicit SystemLoaderPrivate() = default;

  //////////////////////////////////////////////////
  public: std::list<std::string> PluginPaths() const
  {
    gz::common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv(pluginPathEnv);

    // Also add GZ_SYSTEM_SIM_PLUGIN_PATH for compatibility with Garden and
    // later.
    for (const auto &path :
         common::SystemPaths::PathsFromEnv(this->pluginPathEnvGzSim))
    {
      systemPaths.AddPluginPaths(path);
    }

    for (const auto &path : this->systemPluginPaths)
      systemPaths.AddPluginPaths(path);

    std::string homePath;
    gz::common::env(IGN_HOMEDIR, homePath);
    systemPaths.AddPluginPaths(common::joinPaths(
        homePath, ".ignition", "gazebo", "plugins"));
    systemPaths.AddPluginPaths(IGN_GAZEBO_PLUGIN_INSTALL_DIR);

    return systemPaths.PluginPaths();
  }

  //////////////////////////////////////////////////
  public: bool InstantiateSystemPlugin(const sdf::Plugin &_sdfPlugin,
              ignition::plugin::PluginPtr &_gzPlugin)
  {
    const std::string gzSimPrefix{"gz-sim"};
    auto filename = _sdfPlugin.Filename();
    auto pos = filename.find(gzSimPrefix);
    if (pos == 0u)
    {
      filename.replace(pos, gzSimPrefix.size(), "ignition-gazebo");
    }

    std::list<std::string> paths = this->PluginPaths();
    common::SystemPaths systemPaths;
    for (const auto &p : paths)
    {
      systemPaths.AddPluginPaths(p);
    }

    auto pathToLib = systemPaths.FindSharedLibrary(filename);
    if (pathToLib.empty())
    {
      // We assume gz::sim corresponds to the levels feature
      if (_sdfPlugin.Name() != "gz::sim")
      {
        ignerr << "Failed to load system plugin [" << _sdfPlugin.Filename() <<
                  "] : couldn't find shared library." << std::endl;
      }
      return false;
    }

    auto pluginNames = this->loader.LoadLib(pathToLib);
    if (pluginNames.empty())
    {
      ignerr << "Failed to load system plugin [" << _sdfPlugin.Filename() <<
                "] : couldn't load library on path [" << pathToLib <<
                "]." << std::endl;
      return false;
    }

    auto pluginName = *pluginNames.begin();
    if (pluginName.empty())
    {
      ignerr << "Failed to load system plugin [" << _sdfPlugin.Filename() <<
                "] : couldn't load library on path [" << pathToLib <<
                "]." << std::endl;
      return false;
    }

    // use the first plugin name in the library if not specified
    std::string pluginToInstantiate = _sdfPlugin.Name().empty() ?
        pluginName : _sdfPlugin.Name();

    _gzPlugin = this->loader.Instantiate(pluginToInstantiate);
    if (!_gzPlugin)
    {
      ignerr << "Failed to load system plugin [" << _sdfPlugin.Name() <<
        "] : could not instantiate from library [" << _sdfPlugin.Filename() <<
        "] from path [" << pathToLib << "]." << std::endl;
      return false;
    }

    if (!_gzPlugin->HasInterface<System>())
    {
      ignerr << "Failed to load system plugin [" << _sdfPlugin.Name() <<
        "] : system not found in library  [" << _sdfPlugin.Filename() <<
        "] from path [" << pathToLib << "]." << std::endl;

      return false;
    }

    this->systemPluginsAdded.insert(_gzPlugin);
    return true;
  }

  // Default plugin search path environment variable. Prefer
  // GZ_SYSTEM_SIM_PLUGIN_PATH for compatibility with future versions of Gazebo.
  public: std::string pluginPathEnv{"IGN_GAZEBO_SYSTEM_PLUGIN_PATH"};
  // Default plugin search path environment variable
  public: std::string pluginPathEnvGzSim{"GZ_SIM_SYSTEM_PLUGIN_PATH"};

  /// \brief Plugin loader instace
  public: gz::plugin::Loader loader;

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
std::list<std::string> SystemLoader::PluginPaths() const
{
  return this->dataPtr->PluginPaths();
}

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
  if (_filename == "")
  {
    ignerr << "Failed to instantiate system plugin: empty argument "
              "[(filename): " << _filename << "] " << std::endl;
    return {};
  }

  sdf::Plugin plugin;
  plugin.Load(_sdf);
  plugin.SetFilename(_filename);
  plugin.SetName(_name);
  return LoadPlugin(plugin);
}

//////////////////////////////////////////////////
std::optional<SystemPluginPtr> SystemLoader::LoadPlugin(
  const sdf::ElementPtr &_sdf)
{
  if (nullptr == _sdf)
  {
    return {};
  }
  sdf::Plugin plugin;
  plugin.Load(_sdf);
  return LoadPlugin(plugin);
}

//////////////////////////////////////////////////
std::optional<SystemPluginPtr> SystemLoader::LoadPlugin(
    const sdf::Plugin &_plugin)
{
  ignition::plugin::PluginPtr plugin;

  if (_plugin.Filename() == "")
  {
    ignerr << "Failed to instantiate system plugin: empty argument "
              "[(filename): " << _plugin.Filename() << "] " << std::endl;
    return {};
  }

  auto ret = this->dataPtr->InstantiateSystemPlugin(_plugin, plugin);
  if (ret && plugin)
    return plugin;
  return {};
}

//////////////////////////////////////////////////
std::string SystemLoader::PrettyStr() const
{
  return this->dataPtr->loader.PrettyStr();
}
