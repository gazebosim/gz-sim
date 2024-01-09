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

#ifndef GZ_SIM_SYSTEMS_TOUCH_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_TOUCH_PLUGIN_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class TouchPluginPrivate;

  /// \brief Plugin which publishes a message if the model it is attached
  /// to has touched one or more specified targets continuously during a
  /// given time.
  ///
  /// After publishing, the plugin is disabled. It can be re-enabled through
  /// a Gazebo Transport service call.
  ///
  /// The plugin requires that a contact sensors is placed in at least one
  /// link on the model on which this plugin is attached.
  ///
  /// ## System Parameters
  ///
  /// - `<target>` Name, or substring of a name, that identifies the target
  ///              collision entity/entities.
  ///              This value is searched in the scoped name of all collision
  ///              entities, so it can possibly match more than one collision.
  ///              For example, using the name of a model will match all of its
  ///              collisions (scoped name
  ///              `/model_name/link_name/collision_name`).
  ///
  /// - `<time>` Target time in seconds to maintain contact.
  ///
  /// - `<namespace>` Namespace for transport topics/services:
  ///   - `/<namespace>/enable` : Service used to enable and disable
  ///                             the plugin.
  ///   - `/<namespace>/touched` : Topic where a message is published
  ///                              once the touch event occurs.
  ///
  /// - `<enabled>` Set this to true so the plugin works from the start and
  ///               doesn't need to be enabled.
  class TouchPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate,
        public ISystemReset
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
    public: void Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<TouchPluginPrivate> dataPtr;
  };
  }
}
}
}

#endif
