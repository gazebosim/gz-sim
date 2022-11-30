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
#ifndef GZ_SIM_COMPONENTS_JOINTEFFORTLIMITSCMD_HH_
#define GZ_SIM_COMPONENTS_JOINTEFFORTLIMITSCMD_HH_

#include <vector>
#include <gz/math/Vector2.hh>

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {

namespace components
{

/// \brief Command for setting effort limits of a joint. Data are a vector
/// with a Vector2 for each DOF. The X() component of the Vector2 specifies
/// the minimum effort limit, the Y() component stands for maximum limit.
/// Set to +-infinity to disable the limits.
/// \note It is expected that the physics plugin reads this component and
/// sets the limit to the dynamics engine. After setting it, the data of this
/// component will be cleared (i.e. the vector will have length zero).
using JointEffortLimitsCmd = Component<
  std::vector<gz::math::Vector2d>,
  class JointEffortLimitsCmdTag,
  serializers::VectorSerializer<gz::math::Vector2d>
>;

GZ_SIM_REGISTER_COMPONENT(
  "gz_sim_components.JointEffortLimitsCmd", JointEffortLimitsCmd)
}

}
}
}

#endif
