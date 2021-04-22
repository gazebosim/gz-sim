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

  /// \brief This class provides a class that simulates a maritime thruster for
  /// boats and underwater vehicles. It uses the equations described in Fossen's
  /// "Guidance and Control of Ocean Vehicles" in page 246. This plugin takes in
  /// force in Newtons and applies it to the thruster. It also calculates the
  /// theoretical RPM of the blades and spins them at that RPM. The rationale
  /// for directly using force
  ///
  /// ## System Parameters
  /// <namespace> - The namespace in which the robot exists. The plugin will
  ///   listen on the topic `/model/{namespace}/joint/{joint_name}/cmd_pos`.
  /// <joint_name> - This is the joint in the model which corresponds to the
  ///   propeller.
  /// <thrust_coefficient> - This is the coefficient which relates the RPM to
  ///   actual thrust.
  /// <fluid_density> - The fluid density of the liquid in which the thruster
  ///   is operating in.
  /// <propeller_diameter> - The propeller diameter is the diameter of the prop
  ///   in meters.
  ///
  /// # Example
  /// An example configuration is provided in the examples folder. The example
  /// uses the LiftDrag plugin to apply steering controls. It also uses the
  /// thruster plugin to propell the craft and the buoyancy plugin for buoyant
  /// force. To run th example run.
  /// ```
  /// ign gazebo auv_controls.sdf
  /// ```
  /// To control the rudder of the craft run the following
  /// ```
  /// ign topic -t /model/tethys/joint/vertical_fins_joint/0/cmd_pos
  ///    -m ignition.msgs.Double -p 'data: -0.17'
  /// ```
  /// To apply a thrust you may run the following command
  /// The vehicle should move in a circle.
  /// ```
  /// ign topic -t /model/tethys/joint/propeller_joint/cmd_pos
  /// -m ignition.msgs.Double -p 'data: -31'
  /// ```
  class Thruster:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
    public: Thruster();

    /// \brief Destructor
    public: ~Thruster() override;

    /// Documentation inherited
    public: void Configure(
        const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &/*_eventMgr*/);

    /// Documentation inherited
    public: void PreUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Private data pointer
    private: std::unique_ptr<ThrusterPrivateData> dataPtr;
  };
}
}
}
}

#endif
