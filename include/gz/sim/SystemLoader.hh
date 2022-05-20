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
#ifndef GZ_GAZEBO_SYSTEMLOADER_HH_
#define GZ_GAZEBO_SYSTEMLOADER_HH_

#include <memory>
#include <optional>
#include <string>

#include <sdf/Element.hh>

#include <gz/sim/Export.hh>
#include <gz/sim/System.hh>
#include <gz/sim/SystemPluginPtr.hh>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class IGNITION_GAZEBO_HIDDEN SystemLoaderPrivate;

    /// \class SystemLoader SystemLoader.hh gz/sim/SystemLoader.hh
    /// \brief Class for loading/unloading System plugins.
    class IGNITION_GAZEBO_VISIBLE SystemLoader
    {
      /// \brief Constructor
      public: explicit SystemLoader();

      /// \brief Destructor
      public: ~SystemLoader();

      /// \brief Add path to search for plugins.
      /// \param[in] _path New path to be added.
      public: void AddSystemPluginPath(const std::string &_path);

      /// \brief Load and instantiate system plugin from an SDF element.
      /// \param[in] _sdf SDF Element describing plugin instance to be loaded.
      /// \returns Shared pointer to system instance or nullptr.
      public: std::optional<SystemPluginPtr> LoadPlugin(
                  const sdf::ElementPtr &_sdf);

      /// \brief Load and instantiate system plugin from name/filename.
      /// \param[in] _filename Shared library filename to load plugin from.
      /// \param[in] _name Class name to be instantiated.
      /// \param[in] _sdf SDF Element describing plugin instance to be loaded.
      /// \returns Shared pointer to system instance or nullptr.
      public: std::optional<SystemPluginPtr> LoadPlugin(
                  const std::string &_filename,
                  const std::string &_name,
                  const sdf::ElementPtr &_sdf);

      /// \brief Makes a printable string with info about systems
      /// \returns A pretty string
      public: std::string PrettyStr() const;

      /// \brief Pointer to private data.
      private: std::unique_ptr<SystemLoaderPrivate> dataPtr;
    };
    }
    using SystemLoaderPtr = std::shared_ptr<SystemLoader>;
  }
}
#endif  // GZ_GAZEBO_SYSTEMLOADER_HH_

