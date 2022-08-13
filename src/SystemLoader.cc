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

class gz::sim::SystemLoaderPrivate
{
  //////////////////////////////////////////////////
  public: explicit SystemLoaderPrivate() = default;

  //////////////////////////////////////////////////
  public: bool InstantiateSystemPlugin(const sdf::Plugin &_sdfPlugin,
              gz::plugin::PluginPtr &_gzPlugin)
  {
    // Deprecated: accept ignition-gazebo-prefixed systems
    std::string deprecatedPrefix{"ignition-gazebo"};
    auto filename = _sdfPlugin.Filename();
    auto pos = filename.find(deprecatedPrefix);
    if (pos != std::string::npos)
    {
      filename.replace(pos, deprecatedPrefix.size(), "gz-sim");
      gzwarn << "Trying to load deprecated plugin [" << _sdfPlugin.Filename()
             << "]. Using [" << filename << "] instead." << std::endl;
    }

    gz::common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv(pluginPathEnv);

    for (const auto &path : this->systemPluginPaths)
      systemPaths.AddPluginPaths(path);

    std::string homePath;
    gz::common::env(GZ_HOMEDIR, homePath);
    systemPaths.AddPluginPaths(homePath + "/.gz/sim/plugins");

    // TODO(CH3): Deprecated. Remove on tock.
    systemPaths.AddPluginPaths(homePath + "/.ignition/gazebo/plugins");

    systemPaths.AddPluginPaths(GZ_SIM_PLUGIN_INSTALL_DIR);

    auto pathToLib = systemPaths.FindSharedLibrary(filename);
    if (pathToLib.empty())
    {
      // Try deprecated environment variable
      // TODO(CH3): Deprecated. Remove on tock.
      common::SystemPaths systemPathsDep;
      systemPathsDep.SetPluginPathEnv(pluginPathEnvDeprecated);
      pathToLib = systemPathsDep.FindSharedLibrary(filename);

      if (pathToLib.empty())
      {
        // We assume gz::sim corresponds to the levels feature
        if (_sdfPlugin.Name() != "gz::sim")
        {
          gzerr << "Failed to load system plugin [" << filename <<
                    "] : couldn't find shared library." << std::endl;
        }
        return false;
      }
      else
      {
        gzwarn << "Found plugin [" << filename
               << "] using deprecated environment variable ["
               << pluginPathEnvDeprecated << "]. Please use ["
               << pluginPathEnv << "] instead." << std::endl;
      }
    }

    auto pluginNames = this->loader.LoadLib(pathToLib, true);
    if (pluginNames.empty())
    {
      gzerr << "Failed to load system plugin [" << filename <<
                "] : couldn't load library on path [" << pathToLib <<
                "]." << std::endl;
      return false;
    }

    auto pluginName = *pluginNames.begin();
    if (pluginName.empty())
    {
      gzerr << "Failed to load system plugin [" << filename <<
                "] : couldn't load library on path [" << pathToLib <<
                "]." << std::endl;
      return false;
    }

    _gzPlugin = this->loader.Instantiate(_sdfPlugin.Name());
    if (!_gzPlugin)
    {
      gzerr << "Failed to load system plugin [" << _sdfPlugin.Name() <<
        "] : could not instantiate from library [" << _sdfPlugin.Filename() <<
        "] from path [" << pathToLib << "]." << std::endl;
      return false;
    }

    if (!_gzPlugin->HasInterface<System>())
    {
      gzerr << "Failed to load system plugin [" << _sdfPlugin.Name() <<
        "] : system not found in library  [" << _sdfPlugin.Filename() <<
        "] from path [" << pathToLib << "]." << std::endl;

      return false;
    }

    return true;
  }

  // Default plugin search path environment variable
  public: std::string pluginPathEnv{"GZ_SIM_SYSTEM_PLUGIN_PATH"};
  public: std::string pluginPathEnvDeprecated{"IGN_GAZEBO_SYSTEM_PLUGIN_PATH"};

  /// \brief Plugin loader instace
  public: gz::plugin::Loader loader;

  /// \brief Paths to search for system plugins.
  public: std::unordered_set<std::string> systemPluginPaths;

  /// \brief System plugins that have instances loaded via the manager.
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
  if (_filename == "" || _name == "")
  {
    gzerr << "Failed to instantiate system plugin: empty argument "
              "[(filename): " << _filename << "] " <<
              "[(name): " << _name << "]." << std::endl;
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
  if (_plugin.Filename() == "" || _plugin.Name() == "")
  {
    gzerr << "Failed to instantiate system plugin: empty argument "
             "[(filename): " << _plugin.Filename() << "] " <<
             "[(name): " << _plugin.Name() << "]." << std::endl;
    return {};
  }

  gz::plugin::PluginPtr plugin;
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
