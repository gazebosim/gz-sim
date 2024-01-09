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

#ifndef GZ_SIM_SYSTEMS_STATE_PUBLISHER_HH_
#define GZ_SIM_SYSTEMS_STATE_PUBLISHER_HH_

#include <memory>
#include <set>
#include <string>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief The JointStatePublisher system publishes joint state information
  /// for a model. The published message type is gz::msgs::Model, and the
  /// publication topic is determined by the `<topic>` parameter.
  ///
  /// By default the JointStatePublisher will publish all joints for
  /// a model. Use the `<joint_name>` system parameter, described below, to
  /// control which joints are published.
  ///
  /// ## System Parameters
  ///
  /// - `<topic>`: Name of the topic to publish to. This parameter is optional,
  /// and if not provided, the joint state will be published to
  /// "/world/<world_name>/model/<model_name>/joint_state".
  ///
  /// - `<joint_name>`: Name of a joint to publish. This parameter can be
  /// specified multiple times, and is optional. All joints in a model will
  /// be published if joint names are not specified.
  class JointStatePublisher
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: JointStatePublisher();

    /// \brief Destructor
    public: ~JointStatePublisher() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &,
        EntityComponentManager &_ecm, EventManager &) override;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Create components for a joint.
    /// \param[in] _ecm The EntityComponentManager.
    /// \param[in] _joint The joint entity to create component for.
    private: void CreateComponents(EntityComponentManager &_ecm,
                                   sim::Entity _joint);

    /// \brief The model
    private: Model model;

    /// \brief The communication node
    private: transport::Node node;

    /// \brief The publisher
    private: std::unique_ptr<transport::Node::Publisher> modelPub;

    /// \brief The joints that will be published.
    private: std::set<Entity> joints;

    /// \brief The topic
    private: std::string topic;
  };
  }
}
}
}

#endif
