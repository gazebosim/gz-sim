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
#ifndef GZ_SIM_SYSTEMS_HYDRODYNAMICS_HH_
#define GZ_SIM_SYSTEMS_HYDRODYNAMICS_HH_

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
  /// Added mass should be specified using the SDF `<fluid_added_mass>`
  /// tag on the link's `<inertial>` element. The physics engine
  /// integrates added mass implicitly (unconditionally stable) and
  /// automatically computes the full non-diagonal Coriolis matrix.
  /// See:
  /// http://sdformat.org/spec?ver=1.11&elem=link#inertial_fluid_added_mass
  ///
  /// When both `<fluid_added_mass>` and an ocean current are present,
  /// this plugin automatically applies a Coriolis correction so that
  /// the Coriolis force uses the relative velocity v_r rather than
  /// the absolute velocity v. This correction is zero when there is
  /// no current.
  ///
  /// Legacy added mass parameters (`<xDotU>`, `<yDotV>`, etc.) are
  /// still supported but deprecated due to conditional stability.
  /// They will be removed in a future release.
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
  /// ### Added Mass (Deprecated)
  ///
  /// Added mass via this plugin is deprecated because the explicit
  /// integration is conditionally stable. Use the SDF
  /// `<fluid_added_mass>` tag instead (see above).
  ///
  /// To suppress the deprecation warning, set `<disable_added_mass>`
  /// to true.
  ///
  /// Legacy parameters (Fossen sign convention, values should be
  /// negative):
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
  ///   * `<disable_coriolis>` - Disable the plugin's legacy Coriolis
  ///     force. Note: does not disable the implicit Coriolis from
  ///     `<fluid_added_mass>`. [bool, default: false]
  ///   * `<disable_added_mass>` - Disable the plugin's legacy added
  ///     mass force. [bool, default: false]
  ///
  /// ## Example
  ///
  /// Recommended configuration using `<fluid_added_mass>` on the
  /// link (handled by the physics engine) and this plugin for damping:
  ///
  /// \code{.xml}
  ///   <link name="base_link">
  ///     <inertial>
  ///       <mass>147.8671</mass>
  ///       <fluid_added_mass>
  ///         <xx>4.876161</xx>
  ///         <yy>126.324739</yy>
  ///         <zz>126.324739</zz>
  ///         <qq>33.46</qq>
  ///         <rr>33.46</rr>
  ///       </fluid_added_mass>
  ///     </inertial>
  ///   </link>
  ///
  ///   <plugin filename="gz-sim-hydrodynamics-system"
  ///           name="gz::sim::systems::Hydrodynamics">
  ///     <link_name>base_link</link_name>
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
  /// gz sim auv_controls.sdf
  /// ```
  ///
  /// ## References
  /// [1] Fossen, T. I. "Guidance and Control of Ocean Vehicles."
  ///     Wiley, 1994.
  /// [2] Fossen, T. I. "Handbook of Marine Craft Hydrodynamics and
  ///     Motion Control." Wiley, 2011.
  class Hydrodynamics:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
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

    /// Documentation inherited
    public: void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<HydrodynamicsPrivateData> dataPtr;
  };
}
}
}
}
#endif
