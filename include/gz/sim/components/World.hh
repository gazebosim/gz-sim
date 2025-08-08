/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_WORLD_HH_
#define GZ_SIM_COMPONENTS_WORLD_HH_

#include <sdf/World.hh>

#include <istream>
#include <ostream>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {

// The following is only needed to keep ABI compatibility
// TODO(azeey) Remove in main
namespace traits
{
template<>
class IsOutStreamable<std::ostream, sdf::World>
{
    public: static constexpr bool value = false; // NOLINT
};

template<>
class IsInStreamable<std::istream, sdf::World>
{
    public: static constexpr bool value = false; // NOLINT
};
}

namespace serializers
{

/// \brief Specialize the DefaultSerializer on sdf::World so we can
/// skip serialization
/// TODO(azeey) Do we ever want to serialize this component?
template <>
class DefaultSerializer<sdf::World>
{
  public:
  static std::ostream &Serialize(std::ostream &_out, const sdf::World &)
  {
    return _out;
  }

  public:
  static std::istream &Deserialize(std::istream &_in, sdf::World &)
  {
    return _in;
  }
};
}

namespace components
{
  /// \brief A component that identifies an entity as being a world.
  using World = Component<NoData, class WorldTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.World", World)

  /// \brief A component that holds the world's SDF DOM
  using WorldSdf = Component<sdf::World, class WorldTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.WorldSdf", WorldSdf)
}
}
}
}

#endif
