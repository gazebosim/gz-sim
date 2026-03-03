/*
 * Copyright (C) 2026 Jiayi Cai
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
#ifndef GZ_SIM_SYSTEMS_SWERVEDRIVE_HH_
#define GZ_SIM_SYSTEMS_SWERVEDRIVE_HH_

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
  class SwerveDrivePrivate;

  /// \brief Swerve drive controller which can be attached to a model
  /// with exactly one rotational joint and one steering joint for each of
  /// the four wheels (front-left, front-right, back-left, and back-right).
  ///
  /// ## System Parameters
  ///
  /// - `<front_left_joint>`: Name of a joint that controls a front left wheel.
  /// This element must appear exactly once.
  ///
  /// - `<front_right_joint>`: Name of a joint that controls a front right
  /// wheel. This element must appear exactly once.
  ///
  /// - `<back_left_joint>`: Name of a joint that controls a back left wheel.
  /// This element must appear exactly once.
  ///
  /// - `<back_right_joint>`: Name of a joint that controls a back right wheel.
  /// This element must appear exactly once.
  ///
  /// - `<front_left_steering_joint>`: Name of a joint that controls steering
  /// for front left wheel. This element must appear exactly once.
  ///
  /// - `<front_right_steering_joint>`: Name of a joint that controls steering
  /// for front right wheel. This element must appear exactly once.
  ///
  /// - `<back_left_steering_joint>`: Name of a joint that controls steering
  /// for back left wheel. This element must appear exactly once.
  ///
  /// - `<back_right_steering_joint>`: Name of a joint that controls steering
  /// for back right wheel. This element must appear exactly once.
  ///
  /// - `<wheelbase>`: Longitudinal distance between front and back wheels,
  /// in meters. This element is optional, although it is recommended to be
  /// included with an appropriate value. The default value is 0.5m.
  ///
  /// - `<wheel_separation>`: Lateral distance between left and right wheels,
  /// in meters. This element is optional, although it is recommended to be
  /// included with an appropriate value. The default value is 0.5m.
  ///
  /// - `<wheel_radius>`: Wheel radius in meters. This element is optional,
  /// although it is recommended to be included with an appropriate value. The
  /// default value is 0.15m.
  ///
  /// - `<limit_speed_on_steering_err>`: Enable wheel speed limiting when the
  /// steering angle has not yet reached the commanded value. The default
  /// value is true.
  ///
  /// - `<steering_err_threshold>`: Steering angle error threshold in radians.
  /// When `<limit_speed_on_steering_err>` is true and the absolute steering
  /// error exceeds this value, wheel speed scaling is applied. The default
  /// value is pi/10.
  ///
  /// - `<speed_limit_power>`: Exponent used in the cosine-based wheel speed
  /// scaling factor. When `<limit_speed_on_steering_err>` is true, higher
  /// values result in stronger speed reduction as steering error increases.
  /// The default value is 2.
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
  /// transform from `frame_id` to `child_frame_id`. This element if optional,
  ///  and the default value is `/model/{name_of_model}/tf`.
  ///
  /// - `<frame_id>`: Custom `frame_id` field that this system will use as the
  /// origin of the odometry transform in both the `<tf_topic>`
  /// `gz.msgs.Pose_V` message and the `<odom_topic>`
  /// `gz.msgs.Odometry` message. This element if optional, and the
  /// default value is `{name_of_model}/odom`.
  ///
  /// - `<child_frame_id>`: Custom `child_frame_id` that this system will use as
  /// the target of the odometry trasnform in both the `<tf_topic>`
  /// `gz.msgs.Pose_V` message and the `<odom_topic>`
  /// `gz.msgs.Odometry` message. This element if optional,
  /// and the default value is `{name_of_model}/{name_of_link}`.
  class SwerveDrive
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: SwerveDrive();

    /// \brief Destructor
    public: ~SwerveDrive() override = default;

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
    private: std::unique_ptr<SwerveDrivePrivate> dataPtr;
  };
  }
}
}
}

#endif
