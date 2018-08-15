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

#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class SystemManagerPrivate;

    /// \class SystemManager SystemManager.hh ignition/gazebo/SystemManager.hh
    /// \brief Class for loading/unloading System plugins.
    class IGNITION_GAZEBO_VISIBLE SystemManager
    {
      /// \brief Constructor
      public: explicit SystemManager();

      /// \brief Destructor
      public: ~SystemManager();

      /// \brief Add path to search for plugins.
      public: void addSystemPluginPath(const std::string& _path);

      /// \brief Load system configuration.
      public: bool loadSystemConfig(const std::string& _config);

      /// \brief Load system configuration.
      public: void listPlugins() const;

      private: std::unique_ptr<SystemManagerPrivate> dataPtr;
    };
    }
  }
}
#endif

