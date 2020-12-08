#ifndef IGNITION_GAZEBO_SYSTEMS_JOINT_TRAJECTORY_CONTROLLER_HH_
#define IGNITION_GAZEBO_SYSTEMS_JOINT_TRAJECTORY_CONTROLLER_HH_

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
  class IGNITION_GAZEBO_HIDDEN JointTrajectoryControllerPrivate;

  /// \brief Joint trajectory controller, which can be attached to a model with
  /// reference to one or more 1-axis joints in order to follow a trajectory.
  ///
  /// Joint trajectories can be sent to this plugin via Ignition Transport.
  /// The default topic name is "/model/${MODEL_NAME}/joint_trajectory".
  /// This topic name can be configured with the `<topic>` system parameter.
  ///
  /// This topic accepts ignition::msgs::JointTrajectory messages that represent
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
  /// This trajectory can be produced by a motion planning framework, e.g. MoveIt2.
  ///
  /// The progress of the current trajectory can be similarly tracked on
  /// another topic whose name is derived as `<topic>_progress`.
  ///
  /// ## System Parameters
  ///
  /// `<topic>` The topic name that this plugin subscribes to. Optional parameter.
  ///  Defaults to "/joint_trajectory".
  ///
  /// `<use_header_start_time>` Trajectory starts based on header stamps. Optional parameter.
  ///  Defaults to false.
  ///
  /// `<joint_names>` Joints to control. Optional parameter.
  ///  Separate values by whitespace (e.g. "panda_joint1 panda_joint2 ...")
  ///  Defaults to all joints contained in SDF with their respective order.
  ///
  /// `<initial_positions>` Initial joint positions. Optionald parameter.
  ///  Separate values by whitespace (e.g. "0 1.57 ..."). Follows `joint_names` order.
  ///  Defaults to 0 for all joints.
  ///
  /// `<%s_p_gain>` The proportional gain of the PID. Optional parameter.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_p_gain").
  ///  Separate values by whitespace (e.g. "10.0 30.0 ..."). Follows `joint_names` order.
  ///  The default value is 0 (disabled).
  ///
  /// `<%s_i_gain>` The integral gain of the PID. Optional parameter.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_i_gain").
  ///  Separate values by whitespace (e.g. "1.0 3.0 ..."). Follows `joint_names` order.
  ///  The default value is 0 (disabled).
  ///
  /// `<%s_d_gain>` The derivative gain of the PID. Optional parameter.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_d_gain").
  ///  Separate values by whitespace (e.g. "0.1 0.3 ..."). Follows `joint_names` order.
  ///  The default value is 0 (disabled).
  ///
  /// `<%s_i_max>` The integral upper limit of the PID. Optional parameter.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_i_max").
  ///  Separate values by whitespace (e.g. "1 0.75 ..."). Follows `joint_names` order.
  ///  The default value is -1 (no limit if lower than `%s_i_min`).
  ///
  /// `<%s_i_min>` The integral lower limit of the PID. Optional parameter.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_i_min").
  ///  Separate values by whitespace (e.g. "-1.33 0 ..."). Follows `joint_names` order.
  ///  The default value is 0 (no limit if higher than `%s_i_max`).
  ///
  /// `<%s_cmd_max>` Output max value of the PID. Optional parameter.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_cmd_max").
  ///  Separate values by whitespace (e.g. "1000 5 ..."). Follows `joint_names` order.
  ///  The default value is -1 (no limit if lower than `%s_i_min`).
  ///
  /// `<%s_cmd_min>` Output min value of the PID. Optional parameter.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_cmd_min").
  ///  Separate values by whitespace (e.g. "-1000 -10 ..."). Follows `joint_names` order.
  ///  The default value is 0 (no limit if higher than `%s_i_max`).
  ///
  /// `<%s_cmd_offset>` Command offset (feed-forward) of the PID. Optional parameter.
  ///  Substitute '%s' for "position" or "velocity" (e.g. "position_cmd_offset").
  ///  Separate values by whitespace (e.g. "0.1 0 ..."). Follows `joint_names` order.
  ///  The default value is 0 (no offset).
  class IGNITION_GAZEBO_VISIBLE JointTrajectoryController
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
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                           ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<JointTrajectoryControllerPrivate> dataPtr;
  };
} // namespace systems
} // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
} // namespace gazebo
} // namespace ignition

#endif
