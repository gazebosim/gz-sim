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

#ifndef IGNITION_GAZEBO_SYSTEMS_STATE_PUBLISHER_HH_
#define IGNITION_GAZEBO_SYSTEMS_STATE_PUBLISHER_HH_

#include <memory>
#include <ignition/gazebo/Model.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \brief The JointStatePub system publishes state information for
  /// a model. The published message type is ignition::msgs::Model, and the
  /// publication topic is "/world/<world_name>/model/<model_name>/state".
  class IGNITION_GAZEBO_VISIBLE JointStatePublisher
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

    /// \brief The model
    private: Model model;

    /// \brief The communication node
    private: transport::Node node;

    /// \brief The publisher
    private: std::unique_ptr<transport::Node::Publisher> modelPub;
  };
  }
}
}
}

#endif
