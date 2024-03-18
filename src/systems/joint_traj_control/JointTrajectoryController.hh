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

#ifndef GZ_SIM_SYSTEMS_JOINT_TRAJECTORY_CONTROLLER_HH_
#define GZ_SIM_SYSTEMS_JOINT_TRAJECTORY_CONTROLLER_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class JointTrajectoryControllerPrivate;

  /// \brief Joint trajectory controller, which can be attached to a model with
  /// reference to one or more 1-axis joints in order to follow a trajectory.
  ///
  /// Joint trajectories can be sent to this plugin via Gazebo Transport.
  /// The default topic name is "/model/${MODEL_NAME}/joint_trajectory" that
  /// can be configured with the `<topic>` system parameter.
  ///
  /// This topic accepts gz::msgs::JointTrajectory messages that represent
  /// a full trajectory, defined as temporal `points` with their fields ordered
  /// according to `joint_names` field. The fields under `points` are
  /// `positions` - Controlled by position PID controller for each joint
  /// `velocities` - Controlled by velocity PID controller for each joint
  /// `accelerations` - This field is currently ignored
  /// `effort` - Controlled directly for each joint
  /// `time_from_start` - Temporal information about the point
  ///
  /// Forces/torques from `position`, `velocity` and `effort` are summed and
  /// applied to the joint. Each PID controller can be configured with system
  /// parameters described below.
  ///
  /// Input trajectory can be produced by a motion planning framework such as
  /// MoveIt2. For smooth execution of the trajectory, its points should to be
  /// interpolated before sending them via Gazebo Transport (interpolation
  /// might already be implemented in the motion planning framework of your
  /// choice).
  ///
  /// The progress of the current trajectory can be tracked on topic whose name
  /// is derived as `<topic>_progress`. This progress is indicated in the range
  /// of (0.0, 1.0] and is currently based purely on `time_from_start` contained
  /// in the trajectory points.
  ///
  /// ## System Parameters
  ///
  /// - `<topic>` The name of the topic that this plugin subscribes to.
  ///  Optional parameter.
  ///  Defaults to "/model/${MODEL_NAME}/joint_trajectory".
  ///
  /// - `<use_header_start_time>` If enabled, trajectory execution begins at the
  ///  timestamp contained in the header of received trajectory messages.
  ///  Optional parameter.
  ///  Defaults to false.
  ///
  /// - `<joint_name>` Name of a joint to control.
  ///  This parameter can be specified multiple times, i.e. once for each joint.
  ///  Optional parameter.
  ///  Defaults to all 1-axis joints contained in the model SDF (order is kept).
  ///
  /// - `<initial_position>` Initial position of a joint (for position control).
  ///  This parameter can be specified multiple times. Follows joint_name order.
  ///  Optional parameter.
  ///  Defaults to 0 for all joints.
  ///
  /// - `<%s_p_gain>` The proportional gain of the PID.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_p_gain").
  ///  This parameter can be specified multiple times. Follows joint_name order.
  ///  Optional parameter.
  ///  The default value is 0 (disabled).
  ///
  /// - `<%s_i_gain>` The integral gain of the PID. Optional parameter.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_p_gain").
  ///  This parameter can be specified multiple times. Follows joint_name order.
  ///  Optional parameter.
  ///  The default value is 0 (disabled).
  ///
  /// - `<%s_d_gain>` The derivative gain of the PID.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_p_gain").
  ///  This parameter can be specified multiple times. Follows joint_name order.
  ///  Optional parameter.
  ///  The default value is 0 (disabled).
  ///
  /// - `<%s_i_min>` The integral lower limit of the PID.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_p_gain").
  ///  This parameter can be specified multiple times. Follows joint_name order.
  ///  Optional parameter.
  ///  The default value is 0 (no limit if higher than `%s_i_max`).
  ///
  /// - `<%s_i_max>` The integral upper limit of the PID.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_p_gain").
  ///  This parameter can be specified multiple times. Follows joint_name order.
  ///  Optional parameter.
  ///  The default value is -1 (no limit if lower than `%s_i_min`).
  ///
  /// - `<%s_cmd_min>` Output min value of the PID.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_p_gain").
  ///  This parameter can be specified multiple times. Follows joint_name order.
  ///  Optional parameter.
  ///  The default value is 0 (no limit if higher than `%s_i_max`).
  ///
  /// - `<%s_cmd_max>` Output max value of the PID.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_p_gain").
  ///  This parameter can be specified multiple times. Follows joint_name order.
  ///  Optional parameter.
  ///  The default value is -1 (no limit if lower than `%s_i_min`).
  ///
  /// - `<%s_cmd_offset>` Command offset (feed-forward) of the PID.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_p_gain").
  ///  This parameter can be specified multiple times. Follows joint_name order.
  ///  Optional parameter.
  ///  The default value is 0 (no offset).
  class JointTrajectoryController
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: JointTrajectoryController();

    /// \brief Destructor
    public: ~JointTrajectoryController() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<JointTrajectoryControllerPrivate> dataPtr;
  };
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif
