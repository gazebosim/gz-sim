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
#ifndef IGNITION_GAZEBO_SYSTEMS_THRUSTER_HH_
#define IGNITION_GAZEBO_SYSTEMS_THRUSTER_HH_

#include <ignition/gazebo/System.hh>

#include <memory>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class ThrusterPrivateData;

  /// \brief This plugin simulates a maritime thruster for
  /// boats and underwater vehicles. It uses the equations described in Fossen's
  /// "Guidance and Control of Ocean Vehicles" in page 246. This plugin takes in
  /// force in Newtons and applies it to the thruster. It also calculates the
  /// theoretical angular velocity of the blades and spins them accordingly.
  ///
  /// ## System Parameters
  /// - <namespace> - The namespace in which the robot exists. The plugin will
  ///   listen on the topic `/model/{namespace}/joint/{joint_name}/cmd_thrust`
  ///   or on {namespace}/{topic} if {topic} is set.
  ///   [Optional]
  /// - <topic> - The topic for receiving thrust commands. [Optional]
  /// - <joint_name> - This is the joint in the model which corresponds to the
  ///   propeller. [Required]
  /// - <fluid_density> - The fluid density of the liquid in which the thruster
  ///   is operating in. [Optional, kg/m^3, defaults to 1000 kg/m^3]
  /// - <propeller_diameter> - The diameter of the propeller in meters.
  ///   [Optional, m, defaults to 0.02m]
  /// - <thrust_coefficient> - This is the coefficient which relates the angular
  ///   velocity to actual thrust. [Optional, no units, defaults to 1.0]
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
  /// - <max_thrust_cmd> - Maximum thrust command. [Optional,
  ///                      defaults to 1000N]
  /// - <min_thrust_cmd> - Minimum thrust command. [Optional,
  ///                      defaults to -1000N]
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
  ///    -m ignition.msgs.Double -p 'data: -0.17'
  /// ```
  /// To apply a thrust you may run the following command:
  /// ```
  /// ign topic -t /model/tethys/joint/propeller_joint/cmd_thrust
  /// -m ignition.msgs.Double -p 'data: -31'
  /// ```
  /// The vehicle should move in a circle.
  class Thruster:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: Thruster();

    /// Documentation inherited
    public: void Configure(
        const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        EventManager &/*_eventMgr*/) override;

    /// Documentation inherited
    public: void PreUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) override;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
        const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ThrusterPrivateData> dataPtr;
  };
}
}
}
}

#endif
