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
#ifndef IGNITION_GAZEBO_SYSTEM_MANAGER_HH_
#define IGNITION_GAZEBO_SYSTEM_MANAGER_HH_

#include <memory>
#include <string>
#include <unordered_set>

#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/plugin/SpecializedPluginPtr.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class SystemPluginManagerPrivate;

    /// \class SystemPluginManager SystemPluginManager.hh ignition/gazebo/SystemPluginManager.hh
    /// \brief Class for loading/unloading System plugins.
    class IGNITION_GAZEBO_VISIBLE SystemPluginManager
    {
      /// \brief Constructor
      public: explicit SystemPluginManager();

      /// \brief Destructor
      public: ~SystemPluginManager();

      /// \brief Load a library at the given path
      /// \param[in] _pathToLibrary is the path to a libaray
      /// \returns The set of plugins that have been loaded from the library
      public: void LoadLibrary(const std::string &_pathToLibrary);

      /// \brif Get plugin names that are of a particular system type
      public: std::unordered_set<std::string> PluginsByType(const SystemTypeId& _system_type);

      /// \brief Instantiates a plugin for the given plugin name
      ///
      /// \param[in] _plugin name of the plugin to instantiate
      /// \returns Pointer to instantiated plugin
      public: using SystemPluginPtr = ignition::plugin::SpecializedPluginPtr<ignition::gazebo::System>;
      public: SystemPluginPtr Instantiate(const std::string &_pluginName) const;

      private: std::unique_ptr<SystemPluginManagerPrivate> dataPtr;
    };
    }
  }
}
#endif

