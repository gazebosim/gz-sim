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
#ifndef IGNITION_GAZEBO_COMPONENTS_SCENE_HH_
#define IGNITION_GAZEBO_COMPONENTS_SCENE_HH_

#include <ignition/msgs/scene.pb.h>

#include <sdf/Scene.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/config.hh>

namespace sdf
{
/// \brief Stream insertion operator for `sdf::Scene`.
/// \param[in] _out Output stream.
/// \param[in] _scene Scene to stream
/// \return The stream.
inline std::ostream &operator<<(std::ostream &_out, const Scene &_scene)
{
  auto msg = ignition::gazebo::convert<ignition::msgs::Scene>(_scene);
  msg.SerializeToOstream(&_out);
  return _out;
}

/// \brief Stream extraction operator for `sdf::Scene`.
/// \param[in] _in Input stream.
/// \param[out] _scene Scene to populate
/// \return The stream.
inline std::istream &operator>>(std::istream &_in, Scene &_scene)
{
  ignition::msgs::Scene msg;
  msg.ParseFromIstream(&_in);

  _scene = ignition::gazebo::convert<sdf::Scene>(msg);
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
  /// \brief This component holds scene properties of the world.
  using Scene = Component<sdf::Scene, class SceneTag>;
  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.Scene", Scene)
}
}
}
}

#endif
