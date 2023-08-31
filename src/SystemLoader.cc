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
#include <sstream>
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

#include "gz/sim/InstallationDirectories.hh"
#include <gz/sim/config.hh>

using namespace gz::sim;

class gz::sim::SystemLoaderPrivate
{
  //////////////////////////////////////////////////
  public: explicit SystemLoaderPrivate() = default;

  //////////////////////////////////////////////////
  public: std::list<std::string> PluginPaths() const
  {
    common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv(pluginPathEnv);

    for (const std::string &path : this->systemPluginPaths)
      systemPaths.AddPluginPaths(path);

    std::string homePath;
    common::env(GZ_HOMEDIR, homePath);
    systemPaths.AddPluginPaths(common::joinPaths(
        homePath, ".gz", "sim", "plugins"));
    systemPaths.AddPluginPaths(gz::sim::getPluginInstallDir());

    return systemPaths.PluginPaths();
  }

  //////////////////////////////////////////////////
  public: bool InstantiateSystemPlugin(const sdf::Plugin &_sdfPlugin,
              gz::plugin::PluginPtr &_gzPlugin)
  {
    // Deprecated: accept ignition-gazebo-prefixed systems. Remove this on
    // gz-sim9
    std::string deprecatedPrefix{"ignition-gazebo"};
    auto filename = _sdfPlugin.Filename();
    auto pos = filename.find(deprecatedPrefix);
    if (pos != std::string::npos)
    {
      filename.replace(pos, deprecatedPrefix.size(), "gz-sim");
      gzwarn << "Trying to load deprecated plugin [" << _sdfPlugin.Filename()
             << "]. Using [" << filename << "] instead." << std::endl;
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
        gzerr << "Failed to load system plugin [" << filename <<
                  "] : Could not find shared library." << std::endl;
      }
      return false;
    }

    auto pluginNames = this->loader.LoadLib(pathToLib, true);
    if (pluginNames.empty())
    {
      std::stringstream ss;
      ss << "Failed to load system plugin: "
         << "(Reason: No plugins detected in library)\n"
         << "- Requested plugin name: [" << _sdfPlugin.Name() << "]\n"
         << "- Requested library name: [" << _sdfPlugin.Filename() << "]\n"
         << "- Resolved library path: [" << pathToLib << "]\n";
      gzerr << ss.str();
      return false;
    }

    auto pluginName = *pluginNames.begin();
    if (pluginName.empty())
    {
      std::stringstream ss;
      ss << "Failed to load system plugin: "
         << "(Reason: No plugins detected in library)\n"
         << "- Requested plugin name: [" << _sdfPlugin.Name() << "]\n"
         << "- Requested library name: [" << _sdfPlugin.Filename() << "]\n"
         << "- Resolved library path: [" << pathToLib << "]\n";
      gzerr << ss.str();
      return false;
    }

    // use the first plugin name in the library if not specified
    std::string pluginToInstantiate = _sdfPlugin.Name().empty() ?
        pluginName : _sdfPlugin.Name();

    // Deprecated: accept ignition plugins. Remove this on gz-sim9
    std::string deprecatedPluginNamePrefix{"ignition::gazebo"};
    pos = pluginToInstantiate.find(deprecatedPluginNamePrefix);
    if (pos != std::string::npos)
    {
      auto origPluginName = pluginToInstantiate;
      pluginToInstantiate.replace(pos, deprecatedPluginNamePrefix.size(),
          "gz::sim");
      gzwarn << "Trying to load deprecated plugin name [" << origPluginName
             << "]. Using [" << pluginToInstantiate << "] instead."
             << std::endl;
    }

    _gzPlugin = this->loader.Instantiate(pluginToInstantiate);
    if (!_gzPlugin)
    {
      std::stringstream ss;
      ss << "Failed to load system plugin: "
         << "(Reason: library does not contain requested plugin)\n"
         << "- Requested plugin name: [" << _sdfPlugin.Name() << "]\n"
         << "- Requested library name: [" << _sdfPlugin.Filename() << "]\n"
         << "- Resolved library path: [" << pathToLib << "]\n"
         << "- Detected Plugins:\n";
      for (const auto &pluginIt : pluginNames)
      {
        ss << "  - " << pluginIt << "\n";
        auto aliases = this->loader.AliasesOfPlugin(pluginIt);
        if (!aliases.empty())
        {
          ss << "\n    aliases:\n";
          for (const auto& alias : aliases)
          {
            ss << "      " << alias << "\n";
          }
        }
      }
      gzerr << ss.str();
      return false;
    }

    if (!_gzPlugin->HasInterface<System>())
    {
      std::stringstream ss;
      ss << "Failed to load system plugin: "
         << "(Reason: plugin does not implement System interface)\n"
         << "- Requested plugin name: [" << _sdfPlugin.Name() << "]\n"
         << "- Requested library name: [" << _sdfPlugin.Filename() << "]\n"
         << "- Resolved library path: [" << pathToLib << "]\n"
         << "- Detected Plugins:\n";
      for (const auto &pluginIt : pluginNames)
      {
        ss << "  - " << pluginIt << "\n";
        auto aliases = this->loader.AliasesOfPlugin(pluginIt);
        if (!aliases.empty())
        {
          ss << "\n    aliases:\n";
          for (const auto& alias : aliases)
          {
            ss << "      " << alias << "\n";
          }
        }
      }
      ss << "- Plugin Interfaces Implemented:\n";
      for (const auto &interfaceIt : this->loader.InterfacesImplemented())
      {
        ss << "  - " << interfaceIt << "\n";
      }
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
    const sdf::Plugin &_plugin)
{
  if (_plugin.Filename() == "")
  {
    gzerr << "Failed to instantiate system plugin: empty argument "
             "[(filename): " << _plugin.Filename() << "] " << std::endl;
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
