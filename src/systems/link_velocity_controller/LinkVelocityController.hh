/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_LINKVELOCITYCONTROLLER_HH_
#define IGNITION_GAZEBO_SYSTEMS_LINKVELOCITYCONTROLLER_HH_

#include <memory>
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
  class LinkVelocityontrollerPrivate;

  /// \brief Link velocity controller which can be attached to a model with a
  /// reference to a single link.
  ///
  /// A new Ignition Transport topic is created to send target link velocity.
  /// The topic name is
  /// "/model/<model_name>/link/<link_name>/<link_index>/cmd_pos"
  ///
  /// This topic accepts ignition::msgs::Double values representing the target
  /// position.
  class IGNITION_GAZEBO_VISIBLE LinkVelocityontroller
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: LinkVelocityController();

    /// \brief Destructor
    public: ~LinkVelocityController() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<LinkVelocityControllerPrivate> dataPtr;
  };
  }
}
}
}

#endif
