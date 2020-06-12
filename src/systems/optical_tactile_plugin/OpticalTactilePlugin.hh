/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_SYSTEMS_OPTICAL_TACTILE_PLUGIN_HH_
#define IGNITION_GAZEBO_SYSTEMS_OPTICAL_TACTILE_PLUGIN_HH_

#include <memory>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace systems
{
    // Forward declaration
    class OpticalTactilePluginPrivate;

    /// \brief Plugin that implements an optical tactile sensor
    ///
    /// It requires that contact sensors be placed in at least one link on the
    /// model on which this plugin is attached.
    ///
    /// Parameters:
    ///
    /// <enabled> Set this to true so the plugin works from the start and
    ///           doesn't need to be enabled.

    class IGNITION_GAZEBO_VISIBLE OpticalTactilePlugin :
        public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemUpdate,
        public ISystemPostUpdate
    {
      /// \brief Constructor
      public: OpticalTactilePlugin();

      /// \brief Destructor
      public: ~OpticalTactilePlugin() override = default;

      // Documentation inherited
      public: void Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &_eventMgr) override;

      /// Documentation inherited
      public: void PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm) override;

      /// Documentation inherited
      public: void Update(const UpdateInfo &_info,
                          EntityComponentManager &_ecm) final;

      // Documentation inherited
      public: void PostUpdate(
            const ignition::gazebo::UpdateInfo &_info,
            const ignition::gazebo::EntityComponentManager &_ecm) override;

      /// \brief Private data pointer
      private: std::unique_ptr<OpticalTactilePluginPrivate> dataPtr;
    };
}
}
}
}

#endif
