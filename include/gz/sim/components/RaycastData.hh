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

#ifndef GZ_SIM_COMPONENTS_RAYCASTDATA_HH_
#define GZ_SIM_COMPONENTS_RAYCASTDATA_HH_

#include <gz/math/Vector3.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>

#include <istream>
#include <ostream>
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
  /// \brief Starting point of the ray in entity frame
  gz::math::Vector3d start;

  /// \brief Ending point of the ray in entity frame
  gz::math::Vector3d end;
};

/// \brief A struct that holds the result of a raycasting operation.
struct RaycastResultInfo
{
  /// \brief The hit point in entity frame
  gz::math::Vector3d point;

  /// \brief The fraction of the ray length at the intersection/hit point.
  double fraction;

  /// \brief The normal at the hit point in entity frame
  gz::math::Vector3d normal;
};

/// @brief A struct that holds the raycasting data, including ray and results
struct RaycastDataInfo
{
  /// @brief The rays to cast from the entity.
  std::vector<RayInfo> rays;

  /// @brief The results of the raycasting.
  std::vector<RaycastResultInfo> results;
};
}

namespace serializers
{
  /// \brief Specialization of DefaultSerializer for RaycastDataInfo
  template<> class DefaultSerializer<components::RaycastDataInfo>
  {
    public: static std::ostream &Serialize(
      std::ostream &_out, const components::RaycastDataInfo &)
    {
      return _out;
    }

    public: static std::istream &Deserialize(
      std::istream &_in, components::RaycastDataInfo &)
    {
      return _in;
    }
  };
}

namespace components
{
/// \brief A component type that contains the rays traced from an entity
/// into a physics world, along with the results of the raycasting operation.
///
/// This component is primarily used for applications that require raycasting.
/// The target application defines the rays, and the physics system plugin
/// updates the raycasting results during each update loop.
using RaycastData = Component<RaycastDataInfo, class RaycastDataTag,
                              serializers::DefaultSerializer<RaycastDataInfo>>;

GZ_SIM_REGISTER_COMPONENT("gz_sim_components.RaycastData", RaycastData)
}
}
}
}
#endif  // GZ_SIM_COMPONENTS_RAYCASTDATA_HH_
