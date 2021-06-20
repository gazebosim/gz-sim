/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_TRACKEDVEHICLE_HH_
#define IGNITION_GAZEBO_SYSTEMS_TRACKEDVEHICLE_HH_

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
  class TrackedVehiclePrivate;

  /// \brief Tracked vehicle controller which can be attached to a model
  /// with any number of left and right tracks. The system should be attached
  /// to a model. Each track has to have a TrackController system configured and
  /// running.
  ///
  /// So far, this system only supports tracks that are parallel along a common
  /// axis (other designs are possible, but not implemented).
  ///
  /// # Examples
  ///
  /// See example usage in world example/tracked_vehicle_simple.sdf .
  ///
  /// # System Parameters
  ///
  /// `<left_track>`: Configuration of a left track link. This element can
  /// appear multiple times, and must appear at least once.
  ///
  /// `<right_track>`: Configuration of a right track link. This element can
  /// appear multiple times, and must appear at least once.
  ///
  /// `<left_track>`, `<right_track>` subelements:
  /// - `<link>`: The link representing the track.
  /// - `<velocity_topic>`: The topic on which the track accepts velocity
  ///     commands (defaults to `/model/${model}/link/${link}/track_cmd_vel`).
  /// - `<center_of_rotation_topic>`: The topic on which the track accepts
  ///     center of rotation commands (defaults to
  ///     `/model/${model}/link/${link}/track_cmd_center_of_rotation`).
  ///
  /// `<tracks_separation>`: Distance between tracks, in meters.
  ///
  /// `<track_height>`: Height of the tracks, in meters (used for computing
  /// odometry).
  ///
  /// `<steering_efficiency>`: Initial steering efficiency. Defaults to 0.5.
  ///
  /// `<debug>` If 1, the system will output debugging info and visualizations.
  ///
  /// `<linear_velocity>`: Limiter of linear velocity of the vehicle. Please
  /// note that the tracks can each have their own speed limitations.
  /// - `<min_velocity>`/`<max_velocity>` Min/max velocity of the vehicle (m/s).
  /// - `<min_acceleration>`/`<max_acceleration>` Min/max acceleration of the
  ///   vehicle (m/s^2).
  /// - `<min_jerk>`/`<max_jerk>` Min/max jerk of the vehicle (m/s^3).
  ///
  /// `<angular_velocity>`: Limiter of angular velocity of the vehicle. Please
  /// note that the tracks can each have their own speed limitations.
  /// - `<min_velocity>`/`<max_velocity>` Min/max velocity of the vehicle
  ///   (rad/s).
  /// - `<min_acceleration>`/`<max_acceleration>` Min/max acceleration of the
  ///   vehicle (rad/s^2).
  /// - `<min_jerk>`/`<max_jerk>` Min/max jerk of the vehicle (rad/s^3).
  ///
  /// `<odom_publish_frequency>`: Odometry publication frequency. This
  /// element is optional, and the default value is 50Hz.
  ///
  /// `<topic>`: Custom topic that this system will subscribe to in order to
  /// receive command velocity messages. This element if optional, and the
  /// default value is `/model/{name_of_model}/cmd_vel`.
  ///
  /// `<steering_efficiency_topic>`: Custom topic that this system will
  /// subscribe to in order to receive steering efficiency messages.
  /// This element if optional, and the default value is
  /// `/model/{name_of_model}/steering_efficiency`.
  ///
  /// `<odom_topic>`: Custom topic on which this system will publish odometry
  /// messages. This element if optional, and the default value is
  /// `/model/{name_of_model}/odometry`.
  ///
  /// `<tf_topic>`: Custom topic on which this system will publish the
  /// transform from `frame_id` to `child_frame_id`. This element if optional,
  ///  and the default value is `/model/{name_of_model}/tf`.
  ///
  /// `<frame_id>`: Custom `frame_id` field that this system will use as the
  /// origin of the odometry transform in both the `<tf_topic>`
  /// `ignition.msgs.Pose_V` message and the `<odom_topic>`
  /// `ignition.msgs.Odometry` message. This element if optional, and the
  /// default value is `{name_of_model}/odom`.
  ///
  /// `<child_frame_id>`: Custom `child_frame_id` that this system will use as
  /// the target of the odometry trasnform in both the `<tf_topic>`
  /// `ignition.msgs.Pose_V` message and the `<odom_topic>`
  /// `ignition.msgs.Odometry` message. This element if optional,
  ///  and the default value is `{name_of_model}/{name_of_link}`.
  class TrackedVehicle
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: TrackedVehicle();

    /// \brief Destructor
    public: ~TrackedVehicle() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<TrackedVehiclePrivate> dataPtr;
  };
  }
}
}
}

#endif
