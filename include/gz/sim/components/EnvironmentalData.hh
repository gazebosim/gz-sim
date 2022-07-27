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

#ifndef GZ_SIM_ENVIRONMENTAL_DATA_HH_
#define GZ_SIM_ENVIRONMENTAL_DATA_HH_

#include <string>

#include <gz/common/DataFrame.hh>
#include <gz/math/TimeVaryingVolumetricGrid.hh>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief A component type that contains a common::DataFrame populated with
  /// environmental data across time and space. This is used for example to
  /// introduce other physical quantities that may be of interest even if not
  /// modelled in simulation.
  using EnvironmentalData = Component<
    common::DataFrame<std::string, math::InMemoryTimeVaryingVolumetricGrid<double>>,
    class EnvironmentalDataTag>;

  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.EnvironmentalData", EnvironmentalData)
}
}
}  // namespace sim
}  // namespace gz

#endif // GZ_SIM_ENVIRONMENTAL_DATA_HH_
