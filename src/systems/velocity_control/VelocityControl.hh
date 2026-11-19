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
#ifndef GZ_SIM_SYSTEMS_VELOCITYCONTROL_HH_
#define GZ_SIM_SYSTEMS_VELOCITYCONTROL_HH_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

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
  class VelocityControlPrivate;

  /// \brief Linear and angular velocity controller
  /// which is directly set on a model.
  ///
  /// ## System Parameters
  ///
  /// - `<topic>`: Topic to receive commands in. Defaults to
  ///   `/model/<model_name>/cmd_vel`. without a namespace,
  ///   or `{namespace}/cmd_vel` with a namespace.
  ///   Relative topic names are prefixed with the model namespace when
  ///   specified, while absolute topic names remain unchanged.
  ///
  /// - `<initial_linear>`: Linear velocity to start with.
  ///
  /// - `<initial_angular>`: Linear velocity to start with.
  ///
  /// - `<link_name>`: Deprecated. Use `<link>` instead. Name of a link to
  ///   control. This element can be specified multiple times. The command
  ///   topic defaults to `/model/<model_name>/link/<link_name>/cmd_vel`
  ///   without a namespace, or `{namespace}/link/<link_name>/cmd_vel` with
  ///   a namespace. Custom topic names are not supported.
  ///
  /// - `<link>`: Configuration for controlling an individual link.
  ///   Multiple `<link>` elements can be specified.
  ///   - `<name>`: Name of the link to control.
  ///   - `<topic>`: Topic to receive velocity commands for the link.
  ///     Defaults to `/model/<model_name>/link/<link_name>/cmd_vel` without
  ///     a namespace, or `{namespace}/cmd_vel` with a namespace.
  ///     Relative topic names are prefixed with the model namespace when
  ///     specified, while absolute topic names remain unchanged.
  class VelocityControl
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Resolved topic names
    public: struct TopicNames
    {
      /// \brief Model command velocity topic name
      public: std::string modelCmdVelTopic;

      /// \brief Link command velocity topic names
      public: std::unordered_map<std::string, std::string> linkCmdVelTopics;
    };

    /// \brief Constructor
    public: VelocityControl();

    /// \brief Destructor
    public: ~VelocityControl() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &/*_info*/,
                gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Get the resolved topic names
    public: TopicNames ResolvedTopicNames() const;

    /// \brief Private data pointer
    private: std::unique_ptr<VelocityControlPrivate> dataPtr;
  };
  }
}
}
}

#endif
