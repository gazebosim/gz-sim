/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_HYDROSTATICPRESSURE_HH_
#define GZ_SIM_SYSTEMS_HYDROSTATICPRESSURE_HH_

#include <memory>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class HydrostaticPressurePrivate;

  /// \brief A model-attached system plugin that publishes hydrostatic
  /// pressure readings (gz::msgs::FluidPressure) for an underwater vehicle.
  ///
  /// Pressure is computed from the configured link's world Z position
  /// relative to a fluid surface using the Fofonoff & Millard (1983)
  /// depth-to-pressure polynomial, which accounts for seawater
  /// compressibility and is accurate to ~0.1% over 0-1000 m. Above the
  /// surface the sensor floors at the configured atmospheric pressure.
  ///
  /// ## System Parameters
  /// - `<link_name>`: Name of the link to track. [optional, string,
  ///   default=base_link]
  /// - `<topic>`: Topic on which to publish gz::msgs::FluidPressure.
  ///   [optional, string, default=/<model_name>/pressure]
  /// - `<frame_id>`: Value placed in the message header's frame_id field.
  ///   [optional, string, default=link_name]
  /// - `<update_rate>`: Publishing rate in Hz. [optional, float,
  ///   default=20]
  /// - `<fluid_density>`: Fluid density in kg/m^3. Used by the F&M
  ///   polynomial. [optional, float, default=1025]
  /// - `<surface_pressure_pa>`: Pressure at the fluid surface in Pa.
  ///   [optional, float, default=101325]
  /// - `<gravity>`: Magnitude of gravity in m/s^2 used by the depth-to-
  ///   pressure model. [optional, float, default=9.80665]
  /// - `<surface_z>`: World Z of the fluid/air interface in meters.
  ///   [optional, float, default=0]
  /// - `<noise_stddev>`: Standard deviation of additive Gaussian noise
  ///   applied to the pressure reading, in Pa. [optional, float,
  ///   default=0]
  ///
  /// ## Topics
  /// - Publishes gz::msgs::FluidPressure on the configured `<topic>`.
  class HydrostaticPressure:
    public System,
    public ISystemConfigure,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: HydrostaticPressure();

    /// \brief Destructor
    public: ~HydrostaticPressure() override;

    // Documentation inherited
    public: void Configure(
        const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
        const UpdateInfo &_info,
        const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<HydrostaticPressurePrivate> dataPtr;
  };
}
}
}
}
#endif
