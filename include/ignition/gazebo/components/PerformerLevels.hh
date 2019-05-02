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
#ifndef IGNITION_GAZEBO_COMPONENTS_PERFORMERLEVELS_HH_
#define IGNITION_GAZEBO_COMPONENTS_PERFORMERLEVELS_HH_

#include <set>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Component.hh"

namespace std
{
/// \brief Stream insertion operator for `std::set<Entity>`.
/// \param[in] _out Output stream.
/// \param[in] _set Set to stream
/// \return The stream.
inline std::ostream &operator<<(std::ostream &_out,
      const std::set<ignition::gazebo::Entity> &_set)
{
  for (const auto &level : _set)
  {
    _out << level << " ";
  }
  return _out;
}

/// \brief Stream extraction operator for `std::set<Entity>`.
/// \param[in] _in Input stream.
/// \param[out] _set Set to populate
/// \return The stream.
inline std::istream &operator>>(std::istream &_in,
    std::set<ignition::gazebo::Entity> &_set)
{
  _in.setf(std::ios_base::skipws);

  _set.clear();

  for (auto it = std::istream_iterator<ignition::gazebo::Entity>(_in);
      it != std::istream_iterator<ignition::gazebo::Entity>(); ++it)
  {
    _set.insert(*it);
  }
  return _in;
}
}

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief Holds all the levels which a performer is in.
  using PerformerLevels =
      Component<std::set<Entity>, class PerformerLevelsTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.PerformerLevels",
      PerformerLevels)
}
}
}
}
#endif

