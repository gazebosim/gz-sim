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
#ifndef GZ_SIM_COMPONENTS_DETACHABLE_JOINT_HH_
#define GZ_SIM_COMPONENTS_DETACHABLE_JOINT_HH_

#include <string>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief Data structure to hold information about the parent and child links
  /// connected by a detachable joint
  struct DetachableJointInfo
  {
    /// \brief Entity of the parent link
    Entity parentLink;
    /// \brief Entity of the echild link
    Entity childLink;
    // \brief Type of joint. Only the "fixed" joint type is currently supported.
    std::string jointType = {"fixed"};

    public: bool operator==(const DetachableJointInfo &_info) const
    {
      return (this->parentLink == _info.parentLink) &&
             (this->childLink == _info.childLink) &&
             (this->jointType == _info.jointType);
    }

    public: bool operator!=(const DetachableJointInfo &_info) const
    {
      return !(*this == _info);
    }
  };
}

namespace serializers
{
  /// \brief Serializer for DetachableJointInfo object
  class DetachableJointInfoSerializer
  {
    /// \brief Serialization for `DetachableJointInfo`.
    /// \param[in] _out Output stream.
    /// \param[in] _info DetachableJointInfo object to stream
    /// \return The stream.
    public: static std::ostream &Serialize(
                std::ostream &_out,
                const components::DetachableJointInfo &_info)
    {
      _out << _info.parentLink << " " << _info.childLink << " "
           << _info.jointType;
      return _out;
    }

    /// \brief Deserialization for `std::set<std::string>`.
    /// \param[in] _in Input stream.
    /// \param[in] _info DetachableJointInfo object to populate
    /// \return The stream.
    public: static std::istream &Deserialize(
                std::istream &_in, components::DetachableJointInfo &_info)
    {
      _in >> _info.parentLink >> _info.childLink >> _info.jointType;
      return _in;
    }
  };
}

namespace components
{
  /// \brief A component that identifies an entity as being a detachable joint.
  /// It also contains additional information about the joint.
  using DetachableJoint =
      Component<DetachableJointInfo, class DetachableJointTag,
                serializers::DetachableJointInfoSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.DetachableJoint",
                                DetachableJoint)
}
}
}
}

#endif
