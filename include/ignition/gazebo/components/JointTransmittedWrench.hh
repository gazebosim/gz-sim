/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_COMPONENTS_JOINTTRANSMITTEDWRENCH_HH_
#define IGNITION_GAZEBO_COMPONENTS_JOINTTRANSMITTEDWRENCH_HH_

#include <array>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

/// \brief Data structure that contains torque and force.
struct Wrench
{
  math::Vector3d torque;
  math::Vector3d force;

  /// \brief Equality comparison operator
  /// \param[in] _other The other Wrench against which this wrench is compared.
  /// \return True if both the torque and force values are equal.
  bool operator==(const Wrench &_other) const
  {
    return (this->torque == _other.torque) && (this->force == _other.force);
  }
};

namespace serializers
{
class WrenchSerializer
{
  /// \brief Serialization for `Wrench`.
  /// \param[in] _out Output stream.
  /// \param[in] _wrench Wrench to stream
  /// \return The stream.
  public: static std::ostream &Serialize(std::ostream &_out,
                                         const Wrench &_wrench)
  {
    _out << _wrench.torque << " " << _wrench.torque;
    return _out;
  }

  /// \brief Deserialization for `Wrench`.
  /// \param[in] _in Input stream.
  /// \param[out] _Wrench Wrench to populate
  /// \return The stream.
  public: static std::istream &Deserialize(std::istream &_in, Wrench &_wrench)
  {
    _in >> _wrench.torque;
    _in >> _wrench.force;
    return _in;
  }
};
}  // namespace serializers

namespace components
{
/// \brief Joint Transmitted wrench in SI units (Nm for revolute, N for
/// prismatic). The first set of 3 values coorespond to the torque while the
/// second set of 3 correspond to the force. The wrench is expressed in the
/// Joint frame and the force component is applied at the joint origin.
/// \note The term Wrench is used here to mean a 6D vector formed by stacking
/// torque and force vectors. This is different from the Wrench used in screw
/// theory.
using JointTransmittedWrench =
    Component<Wrench, class JointTransmittedWrenchTag,
              serializers::WrenchSerializer>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.JointTransmittedWrench",
                              JointTransmittedWrench)
}  // namespace components
}
}
}

#endif
