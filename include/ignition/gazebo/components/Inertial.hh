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
#ifndef IGNITION_GAZEBO_COMPONENTS_INERTIAL_HH_
#define IGNITION_GAZEBO_COMPONENTS_INERTIAL_HH_

#include <ignition/msgs/inertial.pb.h>
#include <ignition/math/Inertial.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Conversions.hh>

namespace ignition
{
namespace math
{
/// \brief Stream insertion operator for `math::Inertiald`.
/// \param[in] _out Output stream.
/// \param[in] _inertial Inertiald to stream
/// \return The stream.
inline std::ostream &operator<<(std::ostream &_out, const Inertiald &_inertial)
{
  auto msg = gazebo::convert<msgs::Inertial>(_inertial);
  msg.SerializeToOstream(&_out);
  return _out;
}

/// \brief Stream extraction operator for `math::Inertiald`.
/// \param[in] _in Input stream.
/// \param[out] _inertial Inertiald to populate
/// \return The stream.
inline std::istream &operator>>(std::istream &_in, Inertiald &_inertial)
{
  msgs::Inertial msg;
  msg.ParseFromIstream(&_in);

  _inertial = gazebo::convert<math::Inertiald>(msg);
  return _in;
}
}
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief This component holds an entity's inertial.
  using Inertial = Component<math::Inertiald, class InertialTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.Inertial", Inertial)
}
}
}
}

#endif
