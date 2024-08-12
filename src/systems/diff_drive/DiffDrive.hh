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
#ifndef GZ_SIM_SYSTEMS_DIFFDRIVE_HH_
#define GZ_SIM_SYSTEMS_DIFFDRIVE_HH_

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
  class DiffDrivePrivate;

  /// \brief Differential drive controller which can be attached to a model
  /// with any number of left and right wheels.
  ///
  /// ## System Parameters
  ///
  /// - `<left_joint>`: Name of a joint that controls a left wheel. This
  /// element can appear multiple times, and must appear at least once.
  ///
  /// - `<right_joint>`: Name of a joint that controls a right wheel. This
  /// element can appear multiple times, and must appear at least once.
  ///
  /// - `<wheel_separation>`: Distance between wheels, in meters. This element
  /// is optional, although it is recommended to be included with an
  /// appropriate value. The default value is 1.0m.
  ///
  /// - `<wheel_radius>`: Wheel radius in meters. This element is optional,
  /// although it is recommended to be included with an appropriate value. The
  /// default value is 0.2m.
  ///
  /// - `<odom_publish_frequency>`: Odometry publication frequency. This
  /// element is optional, and the default value is 50Hz.
  ///
  /// - `<topic>`: Custom topic that this system will subscribe to in order to
  /// receive command velocity messages. This element if optional, and the
  /// default value is `/model/{name_of_model}/cmd_vel`.
  ///
  /// - `<odom_topic>`: Custom topic on which this system will publish odometry
  /// messages. This element if optional, and the default value is
  /// `/model/{name_of_model}/odometry`.
  ///
  /// - `<tf_topic>`: Custom topic on which this system will publish the
  /// transform from `frame_id` to `child_frame_id`. This element is optional,
  ///  and the default value is `/model/{name_of_model}/tf`.
  ///
  /// - `<frame_id>`: Custom `frame_id` field that this system will use as the
  /// origin of the odometry transform in both the `<tf_topic>`
  /// gz.msgs.Pose_V message and the `<odom_topic>`
  /// gz.msgs.Odometry message. This element if optional, and the
  /// default value is `{name_of_model}/odom`.
  ///
  /// - `<child_frame_id>`: Custom `child_frame_id` that this system will use as
  /// the target of the odometry trasnform in both the `<tf_topic>`
  /// gz.msgs.Pose_V message and the `<odom_topic>`
  /// gz.msgs.Odometry message. This element if optional,
  ///  and the default value is `{name_of_model}/{name_of_link}`.
  ///
  /// - `<min_velocity>`: Sets both the minimum linear and minimum angular
  ///  velocity.
  ///
  /// - `<max_velocity>`: Sets both the maximum linear and maximum angular
  /// velocity.
  ///
  /// - `<min_acceleration>`: Sets both the minimum linear and minimum angular
  /// acceleration.
  ///
  /// - `<max_acceleration>`: Sets both the maximum linear and maximum angular
  /// acceleration.
  ///
  /// - `<min_jerk>`: Sets both the minimum linear and minimum angular jerk.
  ///
  /// - `<max_jerk>`: Sets both the maximum linear and maximum angular jerk.
  ///
  /// - `<min_linear_velocity>`: Sets the minimum linear velocity. Overrides
  /// `<min_velocity>` if set.
  ///
  /// - `<max_linear_velocity>`: Sets the maximum linear velocity. Overrides
  /// `<max_velocity>` if set.
  ///
  /// - `<min_angular_velocity>`: Sets the minimum angular velocity. Overrides
  /// `<min_velocity>` if set.
  ///
  /// - `<max_angular_velocity>`: Sets the maximum angular velocity. Overrides
  /// `<max_velocity>` if set.
  ///
  /// - `<min_linear_acceleration>`: Sets the minimum linear acceleration.
  /// Overrides `<min_acceleration>` if set.
  ///
  /// - `<max_linear_acceleration>`: Sets the maximum linear acceleration.
  /// Overrides `<max_acceleration>` if set.
  ///
  /// - `<min_angular_acceleration>`: Sets the minimum angular acceleration.
  /// Overrides `<min_acceleration>` if set.
  ///
  /// - `<max_angular_acceleration>`: Sets the maximum angular acceleration.
  /// Overrides `<max_acceleration>` if set.
  ///
  /// - `<min_linear_jerk>`: Sets the minimum linear jerk. Overrides
  /// `<min_jerk>` if set.
  ///
  /// - `<max_linear_jerk>`: Sets the maximum linear jerk. Overrides
  /// `<max_jerk>` if set.
  ///
  /// - `<min_angular_jerk>`: Sets the minimum angular jerk. Overrides
  /// `<min_jerk>` if set.
  ///
  /// - `<max_angular_jerk>`: Sets the maximum angular jerk. Overrides
  /// `<max_jerk>` if set.
  class DiffDrive
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: DiffDrive();

    /// \brief Destructor
    public: ~DiffDrive() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<DiffDrivePrivate> dataPtr;
  };
  }
}
}
}

#endif
