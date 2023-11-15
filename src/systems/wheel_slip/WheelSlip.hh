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

#ifndef GZ_SIM_SYSTEMS_WHEELSLIP_HH_
#define GZ_SIM_SYSTEMS_WHEELSLIP_HH_

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
  class WheelSlipPrivate;

  /// \brief A system that updates wheel slip parameters based
  /// on linear wheel spin velocity (radius * spin rate).
  ///
  /// ## System parameters
  ///
  /// It currently assumes that the `fdir1` friction parameter is set
  /// parallel to the joint axis (often [0 0 1]) and that the link
  /// origin is on the joint axis.
  ///
  /// The `slip` parameter is a Force-Dependent Slip (slip1, slip2)
  /// and it has units of velocity / force (m / s / N),
  /// similar to the inverse of a viscous damping coefficient.
  ///
  /// The `slip_compliance` parameters specified in this plugin
  /// are unitless, representing the lateral or longitudinal slip ratio
  /// (see https://en.wikipedia.org/wiki/Slip_(vehicle_dynamics) )
  /// to tangential force ratio (tangential / normal force).
  ///
  /// Note that the maximum force ratio is the friction coefficient.
  /// At each time step, these compliances are multiplied by
  /// the linear wheel spin velocity and divided by the `wheel_normal_force`
  /// parameter specified below in order to match the units of the
  /// slip parameters.
  ///
  /// A graphical interpretation of these parameters is provided below
  /// for a positive value of slip compliance.
  /// The horizontal axis corresponds to the slip ratio at the wheel,
  /// and the vertical axis corresponds to the tangential force ratio
  /// (tangential / normal force).
  ///
  /// As wheel slip increases, the tangential force increases until
  /// it reaches the maximum set by the friction coefficient.
  /// The slip compliance corresponds to the inverse of the slope
  /// of the force before it reaches the maximum value.
  /// A slip compliance of 0 corresponds to a completely vertical
  /// portion of the plot below.
  /// As slip compliance increases, the slope decreases.
  ///
  /** \code{.xml}
        |                                            .
        |      _________ friction coefficient        .
        |     /                                      .
        |    /|                                      .
        |   /-┘ slope is inverse of                  .
        |  /    slip compliance                      .
        | /                                          .
        |/                                           .
      --+-------------------------- slipRatio
        |
  \endcode */
  ///
  /// ## Examples
  ///
  /** \code{.xml}
    <plugin filename="gz-sim-wheel-slip-system"
     name="gz::sim::systems::WheelSlip">
      <wheel link_name="wheel_front_left">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>100</wheel_normal_force>
        <wheel_radius>0.1651</wheel_radius>
      </wheel>
      <wheel link_name="wheel_front_right">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>100</wheel_normal_force>
        <wheel_radius>0.1651</wheel_radius>
      </wheel>
      <wheel link_name="wheel_rear_left">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>80</wheel_normal_force>
        <wheel_radius>0.1651</wheel_radius>
      </wheel>
      <wheel link_name="wheel_rear_right">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>80</wheel_normal_force>
        <wheel_radius>0.1651</wheel_radius>
      </wheel>
    </plugin>
  \endcode */

  class WheelSlip
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: WheelSlip();

    /// \brief Destructor
    public: ~WheelSlip() override = default;

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
    private: std::unique_ptr<WheelSlipPrivate> dataPtr;
  };
  }
}
}
}

#endif
