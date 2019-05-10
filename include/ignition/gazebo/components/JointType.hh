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
namespace serializers
{
  class JointTypeSerializer
  {
    /// \brief Serialization for `sdf::JointType`.
    /// \param[in] _out Output stream.
    /// \param[in] _type JointType to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                                           const sdf::JointType &_type)
    {
      _out << static_cast<int>(_type);
      return _out;
    }

    /// \brief Deserialization for `sdf::JointType`.
    /// \param[in] _in Input stream.
    /// \param[out] _type JointType to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                                             sdf::JointType &_type)
    {
      int type;
      _in >> type;
      _type = sdf::JointType(type);
      return _in;
    }
  };
}
namespace components
{
  /// \brief A component that contains the joint type. This is a simple wrapper
  /// around sdf::JointType
  using JointType = Component<sdf::JointType, class JointTypeTag,
                              serializers::JointTypeSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.JointType", JointType)
}
}
}
}

#endif
