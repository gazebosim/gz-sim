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
#ifndef IGNITION_GAZEBO_COMPONENTS_JOINTAXIS_HH_
#define IGNITION_GAZEBO_COMPONENTS_JOINTAXIS_HH_

#include <ignition/msgs/axis.pb.h>
#include <sdf/JointAxis.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>

namespace sdf
{
/// \brief Stream insertion operator for `sdf::JointAxis`.
/// \param[in] _out Output stream.
/// \param[in] _set Set to stream
/// \return The stream.
inline std::ostream &operator<<(std::ostream &_out, const JointAxis &_axis)
{
  auto msg = ignition::gazebo::convert<ignition::msgs::Axis>(_axis);
  msg.SerializeToOstream(&_out);
  return _out;
}

/// \brief Stream extraction operator for `sdf::JointAxis`.
/// \param[in] _in Input stream.
/// \param[out] _set Set to populate
/// \return The stream.
inline std::istream &operator>>(std::istream &_in, JointAxis &_axis)
{
  ignition::msgs::Axis msg;
  msg.ParseFromIstream(&_in);

  _axis = ignition::gazebo::convert<sdf::JointAxis>(msg);
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
  /// \brief A component that contains the joint axis . This is a simple wrapper
  /// around sdf::JointAxis
  using JointAxis = Component<sdf::JointAxis, class JointAxisTag>;
  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.JointAxis", JointAxis)

  /// \brief A component that contains the second joint axis for joints with two
  /// axes. This is a simple wrapper around sdf::JointAxis
  using JointAxis2 = Component<sdf::JointAxis, class JointAxis2Tag>;
  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.JointAxis2", JointAxis2)
}
}
}
}

#endif
