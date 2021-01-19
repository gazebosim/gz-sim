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
#ifndef IGNITION_GAZEBO_SYSTEMS_POSEPUBLISHER_HH_
#define IGNITION_GAZEBO_SYSTEMS_POSEPUBLISHER_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/pose-publisher-system/Export.hh>
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
  class PosePublisherPrivate;

  /// \brief Pose publisher system. Attach to an entity to publish the
  /// transform of its child entities in the form of ignition::msgs::Pose
  /// messages, or a single ignition::msgs::Pose_V message if
  /// "use_pose_vector_msg" is true.
  ///
  /// The following parameters are used by the system:
  ///
  /// publish_link_pose         : Set to true to publish link pose
  /// publish_visual_pose       : Set to true to publish visual pose
  /// publish_collision_pose    : Set to true to publish collision pose
  /// publish_sensor_pose       : Set to true to publish sensor pose
  /// publish_nested_model_pose : Set to true to publish nested model pose. The
  ///                             pose of the model that contains this system is
  ///                             also published.
  /// use_pose_vector_msg       : Set to true to publish an
  ///                             ignition::msgs::Pose_V message instead of
  ///                             mulitple ignition::msgs::Pose messages.
  /// update_frequency          : Frequency of pose publications in Hz. A
  ///                             negative frequency publishes as fast as
  ///                             possible (i.e, at the rate of the simulation
  ///                             step)
  /// static_publisher          : Set to true to publish static poses on
  ///                             a "<scoped_entity_name>/pose_static" topic.
  ///                             This will cause only dynamic poses to be
  ///                             published on the "<scoped_entity_name>/pose"
  ///                             topic.
  /// static_update_frequency   : Frequency of static pose publications in Hz. A
  ///                             negative frequency publishes as fast as
  ///                             possible (i.e, at the rate of the simulation
  ///                             step).
  class IGNITION_GAZEBO_POSE_PUBLISHER_SYSTEM_VISIBLE PosePublisher
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: PosePublisher();

    /// \brief Destructor
    public: ~PosePublisher() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<PosePublisherPrivate> dataPtr;
  };
  }
}
}
}

#endif
