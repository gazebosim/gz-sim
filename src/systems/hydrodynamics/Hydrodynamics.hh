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
#ifndef IGNITION_GAZEBO_SYSTEMS_HYDRODYNAMICS_HH_
#define IGNITION_GAZEBO_SYSTEMS_HYDRODYNAMICS_HH_

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
  class HydrodynamicsPrivateData;

  /// \brief This class provides hydrodynamic behaviour for underwater vehicles
  /// It is shamelessly based off Brian Bingham's plugin for VRX
  /// which in turn is based of Fossen's equations described in "Guidance and
  /// Control of Ocean Vehicles". The class should be used together with the
  /// buoyancy plugin to help simulate behaviour of maritime vehicles.
  /// Hydrodynamics refers to the behaviour of bodies in water. It includes
  /// forces like linear and quadratic drag, buoyancy (not provided by this
  /// plugin), etc.
  ///
  /// ## System Parameters
  /// The exact description of these parameters can be found on p. 37 of
  /// Fossen's book. They are used to calculate added mass, linear and quadratic
  /// drag and coriolis force.
  ///   <xDotU> - Added mass in x direction
  ///   <yDotV> - Added mass in y direction
  ///   <zDotW> - Added mass in z direction
  ///   <kDotP> - Added mass in roll direction
  ///   <mDotQ> - Added mass in pitch direction
  ///   <nDotR> - Added mass in yaw direction
  ///   <xUU> - Stability derivative, 2nd order, x component
  ///   <xU> - Stability derivative, 1st order, x component
  ///   <yVV> - Stability derivative, 2nd order, y component
  ///   <yV> - Stability derivative, 1st order, y component
  ///   <zWW> - Stability derivative, 2nd order, z component
  ///   <zW> - Stability derivative, 1st order, z component
  ///   <kPP> - Stability derivative, 2nd order, roll component
  ///   <kP> - Stability derivative, 1st order, roll component
  ///   <mQQ> - Stability derivative, 2nd order, pitch component
  ///   <mQ> - Stability derivative, 1st order, pitch component
  ///   <nRR> - Stability derivative, 2nd order, yaw component
  ///   <nR> - Stability derivative, 1st order, yaw component
  /// Additionally the system also supports the following parameters:
  ///   <waterDensity> - The density of the fluid its moving in.
  ///     Defaults to 998kgm^-2.
  ///   <link_name> - The link of the model that is being subject to
  ///     hydrodynamic forces.
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
  /// ign topic -t /model/tethys/joint/vertical_fins_joint/0/cmd_pos \
  ///    -m ignition.msgs.Double -p 'data: -0.17'
  /// ```
  /// To apply a thrust you may run the following command
  /// ```
  /// ign topic -t /model/tethys/joint/propeller_joint/cmd_pos \
  /// -m ignition.msgs.Double -p 'data: -31'
  /// ```
  /// The vehicle should move in a circle.
  class Hydrodynamics:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
  public: Hydrodynamics();

  public: ~Hydrodynamics() override;

  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/
      );

  public: void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm);

  private: std::unique_ptr<HydrodynamicsPrivateData> dataPtr;
  };
}
}
}
}
#endif
