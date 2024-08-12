/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_JOINTCONTROLLER_HH_
#define GZ_SIM_SYSTEMS_JOINTCONTROLLER_HH_

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
  class JointControllerPrivate;

  /// \brief Joint controller which can be attached to a model with a reference
  /// to a single joint. Currently only the first axis of a joint is actuated.
  ///
  /// ## System Parameters
  ///
  /// - `<joint_name>` The name of the joint to control. Required parameter.
  ///  Can also include multiple `<joint_name>` for identical joints.
  ///
  /// - `<use_force_commands>` True to enable the controller implementation
  /// using force commands. If this parameter is not set or is false, the
  /// controller will use velocity commands internally.
  ///
  /// - `<use_actuator_msg>` True to enable the use of actuator message
  /// for velocity command. The actuator msg is an array of floats for
  /// position, velocity and normalized commands. Relies on
  /// `<actuator_number>` for the index of the velocity actuator and
  /// defaults to topic "/actuators" when `topic` or `subtopic not set.
  ///
  /// - `<actuator_number>` used with `<use_actuator_msg>` to set
  /// the index of the velocity actuator.
  ///
  /// - `<topic>` Topic to receive commands in. Defaults to
  ///     `/model/<model_name>/joint/<joint_name>/cmd_vel`.
  ///
  /// - `<sub_topic>` Sub topic to receive commands in.
  /// Defaults to "/model/<model_name>/<sub_topic>".
  ///
  /// - `<initial_velocity>` Velocity to start with.
  ///
  /// ### Velocity mode
  ///
  /// No additional parameters are required.
  ///
  /// ### Force mode
  ///
  /// The controller accepts the next optional parameters:
  ///
  /// - `<p_gain>` The proportional gain of the PID.
  /// The default value is 1.
  ///
  /// - `<i_gain>` The integral gain of the PID.
  /// The default value is 0.
  ///
  /// - `<d_gain>` The derivative gain of the PID.
  /// The default value is 0.
  ///
  /// - `<i_max>` The integral upper limit of the PID.
  /// The default value is 1.
  ///
  /// - `<i_min>` The integral lower limit of the PID.
  /// The default value is -1.
  ///
  /// - `<cmd_max>` Output max value of the PID.
  /// The default value is 1000.
  ///
  /// - `<cmd_min>` Output min value of the PID.
  /// The default value is -1000.
  ///
  /// - `<cmd_offset>` Command offset (feed-forward) of the PID.
  /// The default value is 0.
  class JointController
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: JointController();

    /// \brief Destructor
    public: ~JointController() override = default;

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
    private: std::unique_ptr<JointControllerPrivate> dataPtr;
  };
  }
}
}
}

#endif
