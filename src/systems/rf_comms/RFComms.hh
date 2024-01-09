/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_RFCOMMS_HH_
#define GZ_SIM_SYSTEMS_RFCOMMS_HH_

#include <memory>

#include <gz/utils/ImplPtr.hh>
#include <sdf/Element.hh>
#include "gz/sim/comms/ICommsModel.hh"
#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A comms model that simulates communication using radio frequency
  /// (RF) devices. The model uses a log-distance path loss function.
  ///
  /// This communication model has been ported from:
  /// https://github.com/osrf/subt .
  ///
  /// ## System Parameters
  ///
  /// This system can be configured with the following SDF parameters:
  ///
  /// Optional parameters:
  ///
  /// - `<range_config>`: Element used to capture the range configuration based
  ///   on a log-normal distribution. This block can contain any of the
  ///   next parameters:
  ///   - `<max_range>`: Hard limit on range (meters). No communication will
  ///                    happen beyond this range. Default is 50.
  ///   - `<fading_exponent>`: Fading exponent used in the normal distribution.
  ///                          Default is 2.5.
  ///   - `<l0>`: Path loss at the reference distance (1 meter) in dBm.
  ///             Default is 40.
  ///   - `<sigma>`: Standard deviation of the normal distribution.
  ///                Default is 10.
  ///
  /// - `<radio_config>`: Element used to capture the radio configuration.
  ///                     This block can contain any of the
  ///                     next parameters:
  ///   - `<capacity>`: Capacity of radio in bits-per-second.
  ///                   Default is 54000000 (54 Mbps).
  ///   - `<tx_power>`: Transmitter power in dBm. Default is 27dBm (500mW).
  ///   - `<noise_floor>`: Noise floor in dBm.  Default is -90dBm.
  ///   - `<modulation>`: Supported modulations: ["QPSK"]. Default is "QPSK".
  ///
  /// ## Example
  ///
  /// Here's an example:
  /// ```
  /// <plugin
  ///   filename="gz-sim-rf-comms-system"
  ///   name="gz::sim::systems::RFComms">
  ///   <range_config>
  ///     <max_range>500000.0</max_range>
  ///     <fading_exponent>1.5</fading_exponent>
  ///     <l0>40</l0>
  ///     <sigma>10.0</sigma>
  ///   </range_config>
  ///   <radio_config>
  ///     <capacity>1000000</capacity>
  ///     <tx_power>20</tx_power>
  ///     <noise_floor>-90</noise_floor>
  ///     <modulation>QPSK</modulation>
  ///   </radio_config>
  /// </plugin>
  /// ```
  class RFComms
    : public comms::ICommsModel
  {
    /// \brief Constructor.
    public: explicit RFComms();

    /// \brief Destructor.
    public: ~RFComms() override = default;

    // Documentation inherited.
    public: void Load(const Entity &_entity,
                      std::shared_ptr<const sdf::Element> _sdf,
                      EntityComponentManager &_ecm,
                      EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void Step(const gz::sim::UpdateInfo &_info,
                      const comms::Registry &_currentRegistry,
                      comms::Registry &_newRegistry,
                      EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
  }
}
}
}

#endif
