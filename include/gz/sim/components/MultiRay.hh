/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef GZ_SIM_COMPONENTS_MULTIRAY_HH_
#define GZ_SIM_COMPONENTS_MULTIRAY_HH_

#include <gz/math/Vector3.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>

#include <vector>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
/// \brief A struct that holds the information of a ray.
struct RayInfo
{
  /// \brief Starting point of the ray in world coordinates
  gz::math::Vector3d start;

  /// \brief Ending point of the ray in world coordinates
  gz::math::Vector3d end;
};

/// \brief A struct that holds the results of raycasting.
struct RayIntersectionInfo
{
  /// \brief The hit point in the world coordinates
  gz::math::Vector3d point;

  /// \brief The fraction of the ray length at the intersection/hit point.
  double fraction;

  /// \brief The normal at the hit point in the world coordinates
  gz::math::Vector3d normal;
};

/// \brief A component type that contains multiple rays from an entity.
using MultiRay =
  gz::sim::components::Component<std::vector<RayInfo>, class MultiRayTag>;

GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MultiRay", MultiRay)

/// \brief A component type that contains the raycasting results from multiple
// rays from an entity into a physics world.
using MultiRayIntersections =
  gz::sim::components::Component<std::vector<RayIntersectionInfo>, class MultiRayIntersectionsTag>;

GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MultiRayIntersections", MultiRayIntersections)
}
}
}
}
#endif  // GZ_SIM_COMPONENTS_MULTIRAY_HH_
