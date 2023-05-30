/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_LINEARACCELERATION_HH_
#define GZ_SIM_COMPONENTS_LINEARACCELERATION_HH_

#include <gz/math/Vector3.hh>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

#include <gz/sim/components/Factory.hh>
#include "gz/sim/components/Component.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief A component type that contains linear acceleration of an entity
  /// represented by gz::math::Vector3d.
  using LinearAcceleration =
    Component<math::Vector3d, class LinearAccelerationTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.LinearAcceleration",
      LinearAcceleration)

  /// \brief A component type that contains linear acceleration of an entity
  /// in the world frame represented by gz::math::Vector3d.
  using WorldLinearAcceleration =
      Component<math::Vector3d, class WorldLinearAccelerationTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.WorldLinearAcceleration",
      WorldLinearAcceleration)
}
}
}
}
#endif
