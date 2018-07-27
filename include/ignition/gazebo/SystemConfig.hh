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

#ifndef IGNITION_GAZEBO_SYSTEM_CONFIG_HH_
#define IGNITION_GAZEBO_SYSTEM_CONFIG_HH_

#include <memory>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
  namespace gazebo
  {
    /// \brief Forward declaration
    class SystemConfigPrivate;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    /// \class SystemConfig SystemConfig.hh ignition/gazebo/SystemConfig.hh
    /// \brief ...
    class IGNITION_GAZEBO_VISIBLE SystemConfig
    {
      /// \brief Constructor
      public: explicit SystemConfig(
                  std::shared_ptr<EntityComponentManager> _compMgr);

      public: SystemConfig(const SystemConfig &_config);

      /// \brief Destructor
      public: ~SystemConfig();

      public: EntityComponentManager &EntityComponentMgr() const;

      public: SystemConfig &operator=(const SystemConfig &_config);

      /// \brief Private data pointer
      private: std::unique_ptr<SystemConfigPrivate> dataPtr;
    };
    }
  }
}
#endif
