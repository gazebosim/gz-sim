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
namespace components
{
  /// \brief A derived class `LevelEntityNames` is used below so that the
  /// `*Serialize` functions can be overridden. An alternative would be to
  /// create custom stream operators.
  using LevelEntityNamesBase =
      Component<std::set<std::string>, class LevelEntityNamesTag>;

  /// \brief A component that holds a list of names of entities to be loaded in
  /// a level.
  class LevelEntityNames : public LevelEntityNamesBase
  {
    // Documentation inherited
    public: LevelEntityNames() : LevelEntityNamesBase()
    {
    }

    // Documentation inherited
    public: explicit LevelEntityNames(const std::set<std::string> &_data)
      : LevelEntityNamesBase(_data)
    {
    }

    // Documentation inherited
    public: void Serialize(std::ostream &_out) const override
    {
      for (const auto &level : this->Data())
      {
        _out << level << " ";
      }
    }

    // Documentation inherited
    public: void Deserialize(std::istream &_in) override
    {
      _in.setf(std::ios_base::skipws);

      this->Data().clear();

      for (auto it = std::istream_iterator<std::string>(_in);
          it != std::istream_iterator<std::string>(); ++it)
      {
        this->Data().insert(*it);
      }
    }
  };
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.LevelEntityNames",
      LevelEntityNames)
}
}
}
}
#endif

