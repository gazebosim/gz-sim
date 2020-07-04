/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_JOINTPOSITIONCONTROLLER_HH_
#define IGNITION_GAZEBO_SYSTEMS_JOINTPOSITIONCONTROLLER_HH_

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
  class JointPositionControllerPrivate;

  /// \brief Joint position controller which can be attached to a model with a
  /// reference to a single joint.
  ///
  /// A new Ignition Transport topic is created to send target joint positions.
  /// The topic name is
  /// "/model/<model_name>/joint/<joint_name>/<joint_index>/cmd_pos"
  ///
  /// This topic accepts ignition::msgs::Double values representing the target
  /// position.
  ///
  /// ## System Parameters
  ///
  /// `<joint_name>` The name of the joint to control. Required parameter.
  ///
  /// `<joint_index>` Axis of the joint to control. Optional parameter.
  ///  The default value is 0.
  ///
  /// `<p_gain>` The proportional gain of the PID. Optional parameter.
  ///  The default value is 1.
  ///
  /// `<i_gain>` The integral gain of the PID. Optional parameter.
  ///  The default value is 0.1.
  ///
  /// `<d_gain>` The derivative gain of the PID. Optional parameter.
  ///  The default value is 0.01
  ///
  /// `<i_max>` The integral upper limit of the PID. Optional parameter.
  ///  The default value is 1.
  ///
  /// `<i_min>` The integral lower limit of the PID. Optional parameter.
  ///  The default value is -1.
  ///
  /// `<cmd_max>` Output max value of the PID. Optional parameter.
  ///  The default value is 1000.
  ///
  /// `<cmd_min>` Output min value of the PID. Optional parameter.
  ///  The default value is -1000.
  ///
  /// `<cmd_offset>` Command offset (feed-forward) of the PID. Optional
  /// parameter. The default value is 0.
  class IGNITION_GAZEBO_VISIBLE JointPositionController
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
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<JointPositionControllerPrivate> dataPtr;
  };
  }
}
}
}

#endif
