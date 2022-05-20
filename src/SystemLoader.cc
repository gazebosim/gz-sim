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
  public: bool InstantiateSystemPlugin(const std::string &_filename,
              const std::string &_name,
              const sdf::ElementPtr &/*_sdf*/,
              gz::plugin::PluginPtr &_plugin)
  {
    gz::common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv(pluginPathEnv);

    for (const auto &path : this->systemPluginPaths)
      systemPaths.AddPluginPaths(path);

    std::string homePath;
    gz::common::env(IGN_HOMEDIR, homePath);
    systemPaths.AddPluginPaths(homePath + "/.gz/sim/plugins");
    systemPaths.AddPluginPaths(GZ_SIM_PLUGIN_INSTALL_DIR);

    auto pathToLib = systemPaths.FindSharedLibrary(_filename);
    if (pathToLib.empty())
    {
      // We assume gz::sim corresponds to the levels feature
      if (_name != "gz::sim")
      {
        gzerr << "Failed to load system plugin [" << _filename <<
                  "] : couldn't find shared library." << std::endl;
      }
      return false;
    }

    auto pluginNames = this->loader.LoadLib(pathToLib);
    if (pluginNames.empty())
    {
      gzerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't load library on path [" << pathToLib <<
                "]." << std::endl;
      return false;
    }

    auto pluginName = *pluginNames.begin();
    if (pluginName.empty())
    {
      gzerr << "Failed to load system plugin [" << _filename <<
                "] : couldn't load library on path [" << pathToLib <<
                "]." << std::endl;
      return false;
    }

    _plugin = this->loader.Instantiate(_name);
    if (!_plugin)
    {
      gzerr << "Failed to load system plugin [" << _name <<
                "] : could not instantiate from library [" << _filename <<
                "] from path [" << pathToLib << "]." << std::endl;
      return false;
    }

    if (!_plugin->HasInterface<System>())
    {
      gzerr << "Failed to load system plugin [" << _name <<
        "] : system not found in library  [" << _filename <<
        "] from path [" << pathToLib << "]." << std::endl;

      return false;
    }

    this->systemPluginsAdded.insert(_plugin);
    return true;
  }

  // Default plugin search path environment variable
  public: std::string pluginPathEnv{"GZ_SIM_SYSTEM_PLUGIN_PATH"};

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
  gz::plugin::PluginPtr plugin;

  if (_filename == "" || _name == "")
  {
    gzerr << "Failed to instantiate system plugin: empty argument "
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
