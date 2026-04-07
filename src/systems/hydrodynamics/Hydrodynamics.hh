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

#include <gz/sim/System.hh>
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

  /// \brief Hydrodynamic damping and current forces for underwater
  /// vehicles.
  ///
  /// Computes linear and quadratic damping (drag) forces in the body
  /// frame per Fossen's 6-DOF marine craft model [1][2]. Should be
  /// used together with the Buoyancy plugin. The relevant terms from
  /// Fossen's equation of motion are:
  ///
  ///     D(v_r) * v_r
  ///
  /// where v_r = v - v_current is the velocity relative to the fluid
  /// in the body frame.
  ///
  /// Based on Brian Bingham's
  /// [plugin for VRX](https://github.com/osrf/vrx).
  ///
  /// ## Added Mass and Coriolis
  ///
  /// Added mass and Coriolis forces are also computed by this plugin
  /// via the `<xDotU>`, `<yDotV>`, etc. parameters using explicit
  /// integration. Note that this formulation is only conditionally
  /// stable.
  ///
  /// ## SNAME Naming Convention
  ///
  /// Damping parameters follow the SNAME (Society of Naval Architects
  /// and Marine Engineers) 1950 convention. The first lowercase letter
  /// denotes the force/moment axis, and the remaining uppercase
  /// letters denote the velocity components:
  ///
  /// | Axis | x (surge) | y (sway) | z (heave) | k (roll) | m (pitch)
  /// | n (yaw) |
  /// |------|-----------|----------|-----------|----------|-----------|
  /// |---------|
  /// | Velocity | U | V | W | P | Q | R |
  ///
  /// For example, `<yVabsV>` is the quadratic drag on the sway axis
  /// due to |V|*V, and `<nR>` is the linear yaw damping due to yaw
  /// rate R.
  ///
  /// ## System Parameters
  ///
  /// ### Required
  ///   * `<link_name>` - Link to apply hydrodynamic forces to.
  ///     [string]
  ///
  /// ### Damping (Drag)
  ///
  /// Diagonal terms:
  ///   * `<xU>` - Linear damping, surge [kg/s]
  ///   * `<yV>` - Linear damping, sway [kg/s]
  ///   * `<zW>` - Linear damping, heave [kg/s]
  ///   * `<kP>` - Linear damping, roll [kgm^2/s]
  ///   * `<mQ>` - Linear damping, pitch [kgm^2/s]
  ///   * `<nR>` - Linear damping, yaw [kgm^2/s]
  ///   * `<xUabsU>` - Quadratic abs damping, surge [kg/m]
  ///   * `<yVabsV>` - Quadratic abs damping, sway [kg/m]
  ///   * `<zWabsW>` - Quadratic abs damping, heave [kg/m]
  ///   * `<kPabsP>` - Quadratic abs damping, roll [kgm^2/rad^2]
  ///   * `<mQabsQ>` - Quadratic abs damping, pitch [kgm^2/rad^2]
  ///   * `<nRabsR>` - Quadratic abs damping, yaw [kgm^2/rad^2]
  ///
  /// Cross terms and off-diagonal entries follow the same pattern:
  ///   * Linear: `<{x|y|z|k|m|n}{U|V|W|P|Q|R}>` e.g. `<xR>`
  ///   * Quadratic abs (recommended):
  ///       `<{x|y|z|k|m|n}{U|V|W|P|Q|R}abs{U|V|W|P|Q|R}>`
  ///       e.g. `<xRabsQ>`
  ///   * Quadratic (may cause oscillations):
  ///       `<{x|y|z|k|m|n}{U|V|W|P|Q|R}{U|V|W|P|Q|R}>`
  ///       e.g. `<xRQ>`
  ///
  /// All damping coefficients default to 0.
  ///
  /// ### Added Mass
  ///
  /// Fossen sign convention (values should be negative):
  ///   * `<xDotU>` - Added mass, surge [kg]
  ///   * `<yDotV>` - Added mass, sway [kg]
  ///   * `<zDotW>` - Added mass, heave [kg]
  ///   * `<kDotP>` - Added mass, roll [kgm^2]
  ///   * `<mDotQ>` - Added mass, pitch [kgm^2]
  ///   * `<nDotR>` - Added mass, yaw [kgm^2]
  ///   * Cross terms:
  ///     `<{x|y|z|k|m|n}Dot{U|V|W|P|Q|R}>` e.g. `<xDotR>`
  ///
  /// ### Ocean Current
  ///   * `<default_current>` - Constant current velocity in world
  ///     frame. [gz::math::Vector3d, default: 0 0 0, m/s]
  ///   * `<namespace>` - If set, the plugin subscribes to
  ///     `/model/<namespace>/ocean_current` instead of
  ///     `/ocean_current`. Useful for per-vehicle currents in
  ///     multi-robot scenarios. [string, optional]
  ///
  /// Currents can also be loaded from data files via the
  /// EnvironmentPreload system. If any `lookup_current_*` tag is
  /// present, topic-based currents are ignored:
  ///   * `<lookup_current_x>` - CSV column for x current [string]
  ///   * `<lookup_current_y>` - CSV column for y current [string]
  ///   * `<lookup_current_z>` - CSV column for z current [string]
  ///
  /// ### Feature Flags
  ///   * `<disable_coriolis>` - Disable Coriolis force.
  ///     [bool, default: false]
  ///   * `<disable_added_mass>` - Disable added mass.
  ///     [bool, default: false]
  ///
  /// ## Example
  ///
  /// Typical configuration for an AUV (MBARI Tethys LRAUV):
  ///
  /// \code{.xml}
  ///   <plugin filename="ignition-gazebo-hydrodynamics-system"
  ///           name="ignition::gazebo::systems::Hydrodynamics">
  ///     <link_name>base_link</link_name>
  ///     <xDotU>-4.876161</xDotU>
  ///     <yDotV>-126.324739</yDotV>
  ///     <zDotW>-126.324739</zDotW>
  ///     <kDotP>0</kDotP>
  ///     <mDotQ>-33.46</mDotQ>
  ///     <nDotR>-33.46</nDotR>
  ///     <xUabsU>-6.2282</xUabsU>
  ///     <yVabsV>-601.27</yVabsV>
  ///     <zWabsW>-601.27</zWabsW>
  ///     <kPabsP>-0.1916</kPabsP>
  ///     <mQabsQ>-632.698957</mQabsQ>
  ///     <nRabsR>-632.698957</nRabsR>
  ///   </plugin>
  /// \endcode
  ///
  /// A complete working example is available in
  /// `examples/worlds/auv_controls.sdf`. Run it with:
  /// ```
  /// ign gazebo auv_controls.sdf
  /// ```
  /// To control the rudder of the craft run the following
  /// ```
  /// ign topic -t /model/tethys/joint/vertical_fins_joint/0/cmd_pos
  ///    -m ignition.msgs.Double -p 'data: -0.17'
  /// ```
  /// To apply a thrust you may run the following command
  /// ```
  /// ign topic -t /model/tethys/joint/propeller_joint/cmd_pos
  /// -m ignition.msgs.Double -p 'data: -31'
  /// ```
  /// The vehicle should move in a circle.
  ///
  /// ## Ocean Currents
  /// When underwater, vehicles are often subject to ocean currents. The
  /// hydrodynamics plugin allows simulation of such currents. We can add
  /// a current simply by publishing the following:
  /// ```
  /// ign topic -t /ocean_current -m ignition.msgs.Vector3d -p 'x: 1, y:0, z:0'
  /// ```
  /// You should observe your vehicle slowly drift to the side.
  ///
  /// ## References
  /// [1] Fossen, T. I. "Guidance and Control of Ocean Vehicles."
  ///     Wiley, 1994.
  /// [2] Fossen, T. I. "Handbook of Marine Craft Hydrodynamics and
  ///     Motion Control." Wiley, 2011.
  class Hydrodynamics:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
    public: Hydrodynamics();

    /// \brief Destructor
    public: ~Hydrodynamics() override;

    /// Documentation inherited
    public: void Configure(
        const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &/*_eventMgr*/) override;

    /// Documentation inherited
    public: void PreUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<HydrodynamicsPrivateData> dataPtr;
  };
}
}
}
}
#endif
