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
#ifndef GZ_SIM_COMPONENTS_LEVELENTITYNAMES_HH_
#define GZ_SIM_COMPONENTS_LEVELENTITYNAMES_HH_

#include <iterator>
#include <string>
#include <set>
#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

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
  class LevelEntityNamesSerializer
  {
    /// \brief Serialization for `std::set<std::string>`.
    /// \param[in] _out Output stream.
    /// \param[in] _set Set to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                                           const std::set<std::string> &_set)
    {
      // Character to separate level names. It's the "Unit separator".
      const char sep = 31;

      for (const auto &entity : _set)
      {
        _out << entity << sep;
      }
      return _out;
    }

    /// \brief Deserialization for `std::set<std::string>`.
    /// \param[in] _in Input stream.
    /// \param[out] _set Set to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                                             std::set<std::string> &_set)
    {
      _set.clear();

      const char sep = 31;
      std::string level;
      while (std::getline(_in, level, sep))
      {
        _set.insert(level);
      }
      return _in;
    }
  };
}

namespace components
{
  /// \brief A component that holds a list of names of entities to be loaded in
  /// a level.
  using LevelEntityNames =
      Component<std::set<std::string>, class LevelEntityNamesTag,
                serializers::LevelEntityNamesSerializer>;

  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.LevelEntityNames",
      LevelEntityNames)
}
}
}
}
#endif
