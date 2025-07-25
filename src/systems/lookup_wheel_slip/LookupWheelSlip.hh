/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#ifndef GZ_SIM_SYSTEMS_LOOKUPWHEELSLIP_HH_
#define GZ_SIM_SYSTEMS_LOOKUPWHEELSLIP_HH_

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
  class LookupWheelSlipPrivate;

  /// \brief Lookup Wheel Slip system.
  /// This plugin uses an image lookup map to dynamically adjust the
  /// wheel slip and friction parameters based on the wheel's position on the
  /// target region.
  /// This plugin needs to be used together with the WheelSlip system.
  /// * Lookup map color values and meaning:
  ///   * `128` represents and applies nominal slip/friction.
  ///   * `> 128` applies more slip/friction from nominal
  ///   * `< 128` applies less slip/friction from nominal
  /// * The new slip/friction value is determined by:
  ///   `newCoeff = nominalCoeff + ((pixelColor - nominalColor) * delta)`
  ///   where:
  ///     * `nominalCoeff` is the slip param set in the WheelSlip
  ///       system.
  ///     * `pixelColor` is the color at the current wheel position
  ///     * `nominalColor` is `128`
  ///     * `delta` is either `<slip_compliance_lateral_delta>`,
  ///       `<slip_compliance_longitudinal_delta>`, or `<friction_delta>`
  ///       depending on the color channel
  /// * For wheel friction updates to take effect, the surface's friction needs
  ///   to be set to high (e.g., `100`) since the minimum friction of the
  ///   two contact points (i.e., the wheel and the surface) is used
  /// LookupWheelSlip system params:
  /// * <slip_map>: Image lookup map filename. This needs to be an
  ///   8 bit RGB image map.
  ///   * The red channel updates lateral slip
  ///   * The green channel updates longitudinal slip
  ///   * The blue channel updates friction
  ///   To visualize the lookup map, update model's visual diffuse texture to
  ///   this file, e.g. set this texture to a plane or heightmap.
  ///   If not provided, then wheel slip parameters are not adjusted based on
  ///   any map.
  /// * <size_x>: x size of lookup slip map in meters.
  /// * <size_y>: y size of lookup slip map in meters.
  /// * <wheel_link_name>: **Required** The wheel link name from the
  ///   WheelSlip system. Specify one <wheel_link_name> per wheel link.
  /// * <slip_compliance_lateral>`: The increase/decrease step to be applied to
  ///   the lateral slip. Default is 0.05
  /// * <slip_compliance_longitudinal>: The increase/decrease step to be applied
  ///   to the longitudinal slip. Default is 0.005
  /// * <friction_delta>: The increase/decrease step to be applied to the
  ///   friction coefficients in the primary and secondary directions.
  ///   Default is 0.5
  class LookupWheelSlip
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: LookupWheelSlip();

    /// \brief Destructor
    public: ~LookupWheelSlip() override = default;

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
    private: std::unique_ptr<LookupWheelSlipPrivate> dataPtr;
  };
  }
}
}
}

#endif
