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

#ifndef IGNITION_GAZEBO_SYSTEMS_TOUCH_PLUGIN_HH_
#define IGNITION_GAZEBO_SYSTEMS_TOUCH_PLUGIN_HH_

#include <memory>
#include "ignition/gazebo/touchplugin-system/Export.hh"
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class TouchPluginPrivate;

  /// \brief Plugin which checks if a model has touched some specific target
  /// for a given time continuously and exclusively. After the touch is
  /// completed, the plugin is disabled. It can be re-enabled through an
  /// Ignition transport service.
  ///
  /// It requires that contact sensors be placed in at least one link on the
  /// model on which this plugin is attached.
  ///
  /// Parameters:
  ///
  /// <target> Scoped name of the desired collision entity that is checked to
  ///          see if it's touching this model. This can be a substring of the
  ///          desired collision name so we match more than one collision. For
  ///          example, using the name of a model will match all its collisions.
  ///
  /// <time> Target time in seconds to maintain contact.
  ///
  /// <namespace> Namespace for transport topics/services:
  ///             /<namespace>/enable : Service used to enable and disable the
  ///                                   plugin.
  ///             /<namespace>/touched : Topic where a message is published once
  ///                                    the touch event occurs.
  ///
  /// <enabled> Set this to true so the plugin works from the start and doesn't
  ///           need to be enabled.
  class IGNITION_GAZEBO_TOUCHPLUGIN_SYSTEM_VISIBLE TouchPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: TouchPlugin();

    /// \brief Destructor
    public: ~TouchPlugin() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<TouchPluginPrivate> dataPtr;
  };
  }
}
}
}

#endif
