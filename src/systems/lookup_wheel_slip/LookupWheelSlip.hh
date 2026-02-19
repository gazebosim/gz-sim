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
  /// This plugin dynamically adjusts the wheel slip and friction parameters
  /// based on the wheel's position on the region covered by the specified
  /// lookup image map (slip map) where slip and friction values are encoded in
  /// the image's RGB channels.
  /// This plugin needs to be used together with the WheelSlip system.
  /// LookupWheelSlip system params:
  /// * <slip_map>: (Required) Lookup slip map filename. This needs to be an
  ///   8 bit RGB image.
  ///   * Channels:
  ///     * Red - updates lateral slip
  ///     * Green - updates longitudinal slip
  ///     * Blue - updates friction
  ///   * Pixel values:
  ///     * A pixel value of 128 represents nominal slip / friction, i.e.
  ///       Apply the original slip / friction values from the WheelSlip system.
  ///     * A pixel value of >128 tells the WheelSlip system to apply an
  ///       increased amount of slip / friction values.
  ///     * A pixel value of <128 tells the WheelSlip system to apply a reduced
  ///       amount of slip / friction values.
  ///     * The change in slip / friction to apply to the wheels is computed as:
  ///       (pixel_value - 128) * delta. See the different delta parameters
  ///       below.
  ///   To visualize the lookup map, set the model's visual diffuse texture to
  ///   this file, e.g. set this texture to a plane or heightmap.
  /// * <size_x>: (Required) x size of lookup slip map in meters.
  /// * <size_y>: (Required) y size of lookup slip map in meters.
  /// * <wheel_link_name>: (Required) The wheel link name from the
  ///   WheelSlip system. Specify one <wheel_link_name> per wheel link.
  /// * <slip_compliance_lateral_delta>`: (Optional) The increase / decrease
  ///   step to be applied to the lateral slip. Default is 0.05
  /// * <slip_compliance_longitudinal_delta>: (Optional) The increase / decrease
  ///   step to be applied to the longitudinal slip. Default is 0.005
  /// * <friction_delta>: (Optional) The increase/decrease step to be applied
  ///   to the friction coefficients in the primary and secondary directions.
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
