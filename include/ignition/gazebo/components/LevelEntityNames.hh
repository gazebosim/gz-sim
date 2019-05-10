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
#ifndef IGNITION_GAZEBO_COMPONENTS_LEVELENTITYNAMES_HH_
#define IGNITION_GAZEBO_COMPONENTS_LEVELENTITYNAMES_HH_

#include <iterator>
#include <string>
#include <set>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Component.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
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
      for (const auto &entity : _set)
      {
        _out << entity << " ";
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
      _in.setf(std::ios_base::skipws);

      _set.clear();

      for (auto it = std::istream_iterator<std::string>(_in);
           it != std::istream_iterator<std::string>(); ++it)
      {
        _set.insert(*it);
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

  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.LevelEntityNames",
      LevelEntityNames)
}
}
}
}
#endif

