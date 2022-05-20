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
#ifndef GZ_GAZEBO_SYSTEMS_HYDRODYNAMICS_HH_
#define GZ_GAZEBO_SYSTEMS_HYDRODYNAMICS_HH_

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
  class HydrodynamicsPrivateData;

  /// \brief This class provides hydrodynamic behaviour for underwater vehicles
  /// It is shamelessly based off Brian Bingham's
  /// [plugin for VRX](https://github.com/osrf/vrx).
  /// which in turn is based of Fossen's equations described in "Guidance and
  /// Control of Ocean Vehicles" [1]. The class should be used together with the
  /// buoyancy plugin to help simulate behaviour of maritime vehicles.
  /// Hydrodynamics refers to the behaviour of bodies in water. It includes
  /// forces like linear and quadratic drag, buoyancy (not provided by this
  /// plugin), etc.
  ///
  /// # System Parameters
  /// The exact description of these parameters can be found on p. 37 of
  /// Fossen's book. They are used to calculate added mass, linear and quadratic
  /// drag and coriolis force.
  ///   * <xDotU> - Added mass in x direction [kg]
  ///   * <yDotV> - Added mass in y direction [kg]
  ///   * <zDotW> - Added mass in z direction [kg]
  ///   * <kDotP> - Added mass in roll direction [kgm^2]
  ///   * <mDotQ> - Added mass in pitch direction [kgm^2]
  ///   * <nDotR> - Added mass in yaw direction [kgm^2]
  ///   * <xUU>   - Stability derivative, 2nd order, x component [kg/m]
  ///   * <xU>    - Stability derivative, 1st order, x component [kg]
  ///   * <yVV>   - Stability derivative, 2nd order, y component [kg/m]
  ///   * <yV>    - Stability derivative, 1st order, y component [kg]
  ///   * <zWW>   - Stability derivative, 2nd order, z component [kg/m]
  ///   * <zW>    - Stability derivative, 1st order, z component [kg]
  ///   * <kPP>   - Stability derivative, 2nd order, roll component [kg/m^2]
  ///   * <kP>    - Stability derivative, 1st order, roll component [kg/m]
  ///   * <mQQ>   - Stability derivative, 2nd order, pitch component [kg/m^2]
  ///   * <mQ>    - Stability derivative, 1st order, pitch component [kg/m]
  ///   * <nRR>   - Stability derivative, 2nd order, yaw component [kg/m^2]
  ///   * <nR>    - Stability derivative, 1st order, yaw component [kg/m]
  /// Additionally the system also supports the following parameters:
  ///   * <waterDensity> - The density of the fluid its moving in.
  ///     Defaults to 998kgm^-3. [kgm^-3, deprecated]
  ///   * <water_density> - The density of the fluid its moving in.
  ///     Defaults to 998kgm^-3. [kgm^-3]
  ///   * <link_name> - The link of the model that is being subject to
  ///     hydrodynamic forces. [Required]
  ///   * <namespace> - This allows the robot to have an individual namespace
  ///     for current. This is useful when you have multiple vehicles in
  ///     different locations and you wish to set the currents of each vehicle
  ///     separately. If no namespace is given then the plugin listens on
  ///     the `/ocean_current` topic for a `Vector3d` message. Otherwise it
  ///     listens on `/model/{namespace name}/ocean_current`.[String, Optional]
  ///   * <default_current> - A generic current.
  ///      [vector3d m/s, optional, default = [0,0,0]m/s]
  ///
  /// # Example
  /// An example configuration is provided in the examples folder. The example
  /// uses the LiftDrag plugin to apply steering controls. It also uses the
  /// thruster plugin to propel the craft and the buoyancy plugin for buoyant
  /// force. To run the example run.
  /// ```
  /// ign gazebo auv_controls.sdf
  /// ```
  /// To control the rudder of the craft run the following
  /// ```
  /// ign topic -t /model/tethys/joint/vertical_fins_joint/0/cmd_pos
  ///    -m gz.msgs.Double -p 'data: -0.17'
  /// ```
  /// To apply a thrust you may run the following command
  /// ```
  /// ign topic -t /model/tethys/joint/propeller_joint/cmd_pos
  /// -m gz.msgs.Double -p 'data: -31'
  /// ```
  /// The vehicle should move in a circle.
  ///
  /// ## Ocean Currents
  /// When underwater, vehicles are often subject to ocean currents. The
  /// hydrodynamics plugin allows simulation of such currents. We can add
  /// a current simply by publishing the following:
  /// ```
  /// ign topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: 1, y:0, z:0'
  /// ```
  /// You should observe your vehicle slowly drift to the side.
  ///
  /// # Citations
  /// [1] Fossen, Thor I. _Guidance and Control of Ocean Vehicles_.
  ///    United Kingdom: Wiley, 1994.
  class Hydrodynamics:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: Hydrodynamics();

    /// \brief Destructor
    public: ~Hydrodynamics() override;

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
    private: std::unique_ptr<HydrodynamicsPrivateData> dataPtr;
  };
}
}
}
}
#endif
