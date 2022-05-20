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
#ifndef GZ_GAZEBO_SYSTEMS_THRUSTER_HH_
#define GZ_GAZEBO_SYSTEMS_THRUSTER_HH_

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
  class ThrusterPrivateData;

  /// \brief This plugin simulates a maritime thruster for
  /// boats and underwater vehicles. It uses the equations described in Fossen's
  /// "Guidance and Control of Ocean Vehicles" in page 246. This plugin has two
  /// modes of operation. In the default mode it takes in a
  /// force in Newtons and applies it to the thruster. It also calculates the
  /// theoretical angular velocity of the blades and spins them accordingly.
  /// Alternatively, one may send angular velocity commands to calculate the
  /// force to be applied using the said equation. In the default mode the
  /// plugin will publish angular velocity in radians per second on
  /// `/model/{ns}/joint/{joint_name}/ang_vel` as an gz.msgs.double. If
  /// <use_angvel_cmd> is set to true it publishes force in Newtons instead to
  /// `/model/{ns}/joint/{joint_name}/force`.
  ///
  /// ## System Parameters
  /// - <namespace> - The namespace in which the robot exists. The plugin will
  ///   listen on the topic `/model/{namespace}/joint/{joint_name}/cmd_thrust`or
  ///   `/model/{namespace}/joint/{joint_name}/cmd_vel` depending on the mode of
  ///   operation.
  ///   [Optional]
  /// - <joint_name> - This is the joint in the model which corresponds to the
  ///   propeller. [Required]
  /// - <use_angvel_cmd> - If set to true will make the thruster
  ///   plugin accept commands in angular velocity in radians per seconds in
  ///   terms of newtons. [Optional, Boolean, defaults to false]
  /// - <fluid_density> - The fluid density of the liquid in which the thruster
  ///   is operating in. [Optional, kg/m^3, defaults to 1000 kg/m^3]
  /// - <propeller_diameter> - The diameter of the propeller in meters.
  ///   [Optional, m, defaults to 0.02m]
  /// - <thrust_coefficient> - This is the coefficient which relates the angular
  ///   velocity to thrust. A positive coefficient corresponds to a clockwise
  ///   propeller, which is a propeller that spins clockwise under positive
  ///   thrust when viewed along the parent link from stern (-x) to bow (+x).
  ///   [Optional, no units, defaults to 1.0]
  ///
  ///      omega = sqrt(thrust /
  ///          (fluid_density * thrust_coefficient * propeller_diameter ^ 4))
  ///
  ///   Where omega is the propeller's angular velocity in rad/s.
  /// - <velocity_control> - If true, use joint velocity commands to rotate the
  ///   propeller. If false, use a PID controller to apply wrenches directly to
  ///   the propeller link instead. [Optional, defaults to false].
  /// - <p_gain> - Proportional gain for joint PID controller. [Optional,
  ///              no units, defaults to 0.1]
  /// - <i_gain> - Integral gain for joint PID controller. [Optional,
  ///              no units, defaults to 0.0]
  /// - <d_gain> - Derivative gain for joint PID controller. [Optional,
  ///              no units, defaults to 0.0]
  /// - <max_thrust_cmd> - Maximum input thrust or angular velocity command.
  ///                      [Optional, defaults to 1000N or 1000rad/s]
  /// - <min_thrust_cmd> - Minimum input thrust or angular velocity command.
  ///                      [Optional, defaults to -1000N or -1000rad/s]
  ///
  /// ## Example
  /// An example configuration is installed with Gazebo. The example
  /// uses the LiftDrag plugin to apply steering controls. It also uses the
  /// thruster plugin to propell the craft and the buoyancy plugin for buoyant
  /// force. To run the example:
  /// ```
  /// ign gazebo auv_controls.sdf
  /// ```
  /// To control the rudder of the craft run the following:
  /// ```
  /// ign topic -t /model/tethys/joint/vertical_fins_joint/0/cmd_pos
  ///    -m gz.msgs.Double -p 'data: -0.17'
  /// ```
  /// To apply a thrust you may run the following command:
  /// ```
  /// ign topic -t /model/tethys/joint/propeller_joint/cmd_thrust
  /// -m gz.msgs.Double -p 'data: -31'
  /// ```
  /// The vehicle should move in a circle.
  class Thruster:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: Thruster();

    /// Documentation inherited
    public: void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &/*_eventMgr*/) override;

    /// Documentation inherited
    public: void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ThrusterPrivateData> dataPtr;
  };
}
}
}
}

#endif
