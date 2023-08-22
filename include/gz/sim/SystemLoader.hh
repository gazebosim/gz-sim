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
#ifndef GZ_SIM_SYSTEMLOADER_HH_
#define GZ_SIM_SYSTEMLOADER_HH_

#include <list>
#include <memory>
#include <optional>
#include <string>

#include <sdf/Element.hh>
#include <sdf/Plugin.hh>

#include <gz/sim/Export.hh>
#include <gz/sim/System.hh>
#include <gz/sim/SystemPluginPtr.hh>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Forward declarations.
    class GZ_SIM_HIDDEN SystemLoaderPrivate;

    /// \class SystemLoader SystemLoader.hh gz/sim/SystemLoader.hh
    /// \brief Class for loading/unloading System plugins.
    class GZ_SIM_VISIBLE SystemLoader
    {
      /// \brief Constructor
      public: explicit SystemLoader();

      /// \brief Destructor
      public: ~SystemLoader();

      /// \brief Add path to search for plugins.
      /// \param[in] _path New path to be added.
      public: void AddSystemPluginPath(const std::string &_path);

      /// \brief Load and instantiate system plugin from name/filename.
      /// \param[in] _plugin SDF Plugin to be loaded.
      /// \returns Shared pointer to system instance or nullptr.
      public: std::optional<SystemPluginPtr> LoadPlugin(
                  const sdf::Plugin &_plugin);

      /// \brief Makes a printable string with info about systems
      /// \returns A pretty string
      public: std::string PrettyStr() const;

      /// \brief Get the plugin search paths used for loading system plugins
      /// \return Paths to search for plugins
      public: std::list<std::string> PluginPaths() const;

      /// \brief Pointer to private data.
      private: std::unique_ptr<SystemLoaderPrivate> dataPtr;
    };
    }
    using SystemLoaderPtr = std::shared_ptr<SystemLoader>;
  }
}
#endif  // GZ_SIM_SYSTEMLOADER_HH_
