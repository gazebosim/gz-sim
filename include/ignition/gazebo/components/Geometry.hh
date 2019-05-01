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
#ifndef IGNITION_GAZEBO_COMPONENTS_GEOMETRY_HH_
#define IGNITION_GAZEBO_COMPONENTS_GEOMETRY_HH_

#include <ignition/msgs/geometry.pb.h>

#include <sdf/Geometry.hh>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Conversions.hh>

namespace sdf
{
/// \brief Stream insertion operator for `sdf::Geometry`.
/// \param[in] _out Output stream.
/// \param[in] _geometry Geometry to stream
/// \return The stream.
inline std::ostream &operator<<(std::ostream &_out, const Geometry &_geometry)
{
  auto msg = ignition::gazebo::convert<ignition::msgs::Geometry>(_geometry);
  msg.SerializeToOstream(&_out);
  return _out;
}

/// \brief Stream extraction operator for `sdf::Geometry`.
/// \param[in] _in Input stream.
/// \param[out] _geometry Geometry to populate
/// \return The stream.
inline std::istream &operator>>(std::istream &_in, Geometry &_geometry)
{
  ignition::msgs::Geometry msg;
  msg.ParseFromIstream(&_in);

  _geometry = ignition::gazebo::convert<sdf::Geometry>(msg);
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
  /// \brief This component holds an entity's geometry.
  using Geometry = Component<sdf::Geometry, class GeometryTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.Geometry", Geometry)
}
}
}
}

#endif
