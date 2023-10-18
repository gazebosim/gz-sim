/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 * Copyright (C) 2023 Benjamin Perseghetti, Rudis Laboratories
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
#ifndef GZ_SIM_SYSTEMS_JOINTPOSITIONCONTROLLER_HH_
#define GZ_SIM_SYSTEMS_JOINTPOSITIONCONTROLLER_HH_

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
  class JointPositionControllerPrivate;

  /// \brief Joint position controller which can be attached to a model with a
  /// reference to a single joint.
  ///
  /// A new Gazebo Transport topic is created to send target joint positions.
  /// The default topic name is
  /// "/model/<model_name>/joint/<joint_name>/<joint_index>/cmd_pos".
  ///
  /// This topic accepts gz::msgs::Double values representing the target
  /// position. If you wish to change the topic on which this plugin listens
  /// you may use the `<topic>` parameter to specify which topic the plugin
  /// should listen on.
  ///
  /// ## System Parameters
  ///
  /// - `<joint_name>` The name of the joint to control. Required parameter.
  /// Can also include multiple `<joint_name>` for identical joints.
  ///
  /// - `<joint_index>` Axis of the joint to control. Optional parameter.
  /// The default value is 0.
  ///
  /// - `<use_actuator_msg>` True to enable the use of actutor message
  /// for position command. Relies on `<actuator_number>` for the
  /// index of the position actuator and defaults to topic "/actuators".
  ///
  /// - `<actuator_number>` used with `<use_actuator_commands>` to set
  /// the index of the position actuator.
  ///
  /// - `<p_gain>` The proportional gain of the PID. Optional parameter.
  /// The default value is 1.
  ///
  /// - `<i_gain>` The integral gain of the PID. Optional parameter.
  /// The default value is 0.1.
  ///
  /// - `<d_gain>` The derivative gain of the PID. Optional parameter.
  /// The default value is 0.01
  ///
  /// - `<i_max>` The integral upper limit of the PID. Optional parameter.
  /// The default value is 1.
  ///
  /// - `<i_min>` The integral lower limit of the PID. Optional parameter.
  /// The default value is -1.
  ///
  /// - `<cmd_max>` Output max value of the PID. Optional parameter.
  /// The default value is 1000.
  ///
  /// - `<cmd_min>` Output min value of the PID. Optional parameter.
  /// The default value is -1000.
  ///
  /// - `<cmd_offset>` Command offset (feed-forward) of the PID. Optional
  /// parameter. The default value is 0.
  ///
  /// - `<use_velocity_commands>` Bypasses the PID and creates a perfect
  /// position. The maximum speed on the joint can be set using the `<cmd_max>`
  /// tag.
  ///
  /// - `<topic>` If you wish to listen on a non-default topic you may specify
  /// it here, otherwise the controller defaults to listening on
  /// "/model/<model_name>/joint/<joint_name>/<joint_index>/cmd_pos".
  ///
  /// - `<sub_topic>` If you wish to listen on a sub_topic you may specify it
  /// here "/model/<model_name>/<sub_topic>".
  ///
  /// - `<initial_position>` Initial position of a joint. Optional parameter.
  /// The default value is 0.
  class JointPositionController
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: JointPositionController();

    /// \brief Destructor
    public: ~JointPositionController() override = default;

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
    private: std::unique_ptr<JointPositionControllerPrivate> dataPtr;
  };
  }
}
}
}

#endif
