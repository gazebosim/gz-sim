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
#ifndef GZ_SIM_COMPONENTS_BATTERYPOWERLOAD_HH_
#define GZ_SIM_COMPONENTS_BATTERYPOWERLOAD_HH_

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>

namespace gz::sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief Data structure to hold the consumer power load
  /// and the name of the battery it uses.
  struct BatteryPowerLoadInfo
  {
    /// \brief Entity of the battery to use.
    Entity batteryId;
    /// \brief Battery power load (W) to add to the battery.
    double batteryPowerLoad;
  };

  /// \brief A component that indicates the total consumption of a battery.
  using BatteryPowerLoad =
    Component<BatteryPowerLoadInfo, class BatteryPowerLoadTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.BatteryPowerLoad",
                                BatteryPowerLoad)
}
}
}  // namespace gz::sim

#endif  // GZ_SIM_COMPONENTS_BATTERYPOWERLOAD_HH_
