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
#ifndef GZ_SIM_SYSTEMS_POSEPUBLISHER_HH_
#define GZ_SIM_SYSTEMS_POSEPUBLISHER_HH_

#include <memory>
#include <gz/sim/config.hh>
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
  class PosePublisherPrivate;

  /// \brief Pose publisher system. Attach to an entity to publish the
  /// transform of its child entities in the form of gz::msgs::Pose
  /// messages, or a single gz::msgs::Pose_V message if
  /// "use_pose_vector_msg" is true.
  ///
  /// ## System Parameters
  ///
  /// - `<publish_link_pose>`: Set to true to publish link pose. This element
  ///   is optional, and the default value is true.
  /// - `<publish_visual_pose>`: Set to true to publish visual pose. This
  ///   element is optional, and the default value is false.
  /// - `<publish_collision_pose>`: Set to true to publish collision pose.
  ///   This element is optional, and the default value is false.
  /// - `<publish_sensor_pose>`: Set to true to publish sensor pose. This
  ///   element is optional, and the default value is false.
  /// - `<publish_model_pose>`: Set to true to publish model pose. This
  ///   element is optional. If it is not set, it takes the value of
  ///   `<publish_nested_model_pose>`.
  /// - `<publish_nested_model_pose>`: Set to true to publish nested model
  ///   pose. The pose of the model that contains this system is also published
  ///   unless `<publish_model_pose>` is set to false. This element is
  ///   optional, and the default value is false.
  /// - `<use_pose_vector_msg>`: Set to true to publish a gz::msgs::Pose_V
  ///   message instead of multiple gz::msgs::Pose messages. This element is
  ///   optional, and the default value is false.
  /// - `<update_frequency>`: Frequency of pose publications in Hz. A
  ///   frequency of zero or less publishes as fast as possible (i.e., at the
  ///   rate of the simulation step). This element is optional, and the
  ///   default value is -1.
  /// - `<static_publisher>`: Set to true to publish static poses on a
  ///   "<scoped_entity_name>/pose_static" topic. This will cause only dynamic
  ///   poses to be published on the "<scoped_entity_name>/pose" topic. This
  ///   element is optional, and the default value is false.
  /// - `<static_update_frequency>`: Frequency of static pose publications in
  ///   Hz. A frequency of zero or less publishes as fast as possible (i.e.,
  ///   at the rate of the simulation step). Only used when
  ///   `<static_publisher>` is true. This element is optional. If it is not
  ///   set, it takes the value of `<update_frequency>`.
  /// - `<topic>`: Set a custom topic instead of default value
  class PosePublisher
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
