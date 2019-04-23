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
#ifndef IGNITION_GAZEBO_COMPONENTS_JOINTTYPE_HH_
#define IGNITION_GAZEBO_COMPONENTS_JOINTTYPE_HH_

#include <memory>
#include <sdf/Joint.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief Base class which can be extended to add serialization
  using JointTypeBase = Component<sdf::JointType, class JointTypeTag>;

  /// \brief A component that contains the joint type. This is a simple wrapper
  /// around sdf::JointType
  class JointType : public JointTypeBase
  {
    // Documentation inherited
    public: JointType() : JointTypeBase()
    {
    }

    // Documentation inherited
    public: explicit JointType(const sdf::JointType &_data)
      : JointTypeBase(_data)
    {
    }

    // Documentation inherited
    public: void Serialize(std::ostream &_out) const override
    {
      _out << static_cast<int>(this->Data());
    }

    // Documentation inherited
    public: void Deserialize(std::istream &_in) override
    {
      int type;
      _in >> type;
      this->Data() = sdf::JointType(type);
    }
  };

  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.JointType", JointType)
}
}
}
}

#endif
