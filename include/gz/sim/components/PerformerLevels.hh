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
#ifndef GZ_SIM_COMPONENTS_PERFORMERLEVELS_HH_
#define GZ_SIM_COMPONENTS_PERFORMERLEVELS_HH_

#include <set>
#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/components/Component.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace serializers
{
  class PerformerLevelsSerializer
  {
    /// \brief Serialization for `std::set<Entity>`.
    /// \param[in] _out Output stream.
    /// \param[in] _set Set to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                                           const std::set<Entity> &_set)
    {
      for (const auto &level : _set)
      {
        _out << level << " ";
      }
      return _out;
    }

    /// \brief Deserialization for `std::set<Entity>`.
    /// \param[in] _in Input stream.
    /// \param[out] _set Set to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                                             std::set<Entity> &_set)
    {
      _in.setf(std::ios_base::skipws);

      _set.clear();

      for (auto it = std::istream_iterator<Entity>(_in);
           it != std::istream_iterator<Entity>(); ++it)
      {
        _set.insert(*it);
      }
      return _in;
    }
  };
}

namespace components
{
  /// \brief Holds all the levels which a performer is in.
  using PerformerLevels = Component<std::set<Entity>, class PerformerLevelsTag,
                                    serializers::PerformerLevelsSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.PerformerLevels",
      PerformerLevels)
}
}
}
}
#endif
