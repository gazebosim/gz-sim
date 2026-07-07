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
#ifndef GZ_SIM_COMPONENTS_CHILDREN_HH_
#define GZ_SIM_COMPONENTS_CHILDREN_HH_

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>
#include <gz/sim/detail/flat_set.hh>
#include <gz/sim/Entity.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace serializers
{
  /// \brief Specialize the DefaultSerializer on detail::FlatSet<Entity> so we
  /// can skip serialization
  template <>
  class DefaultSerializer<detail::FlatSet<Entity>>
  {
    public: static std::ostream &Serialize(std::ostream &_out,
                                           const detail::FlatSet<Entity> &)
    {
      return _out;
    }

    public: static std::istream &Deserialize(std::istream &_in,
                                             detail::FlatSet<Entity> &)
    {
      return _in;
    }
  };
}

namespace components
{
  /// \brief This component holds an entity's children entities.
  using Children = Component<detail::FlatSet<Entity>, class ChildrenTag>;
  GZ_SIM_REGISTER_COMPONENT(
      "gz_sim_components.Children", Children)
}
}
}
}

#endif
