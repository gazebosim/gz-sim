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
#ifndef IGNITION_GAZEBO_COMPONENTS_COLLISION_HH_
#define IGNITION_GAZEBO_COMPONENTS_COLLISION_HH_

#include <sdf/Element.hh>

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
  /// \brief A component that identifies an entity as being a collision.
  using Collision = Component<NoData, class CollisionTag>;
  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.Collision", Collision)

  // TODO(anyone) Replace with sdf::Surface. We really want to hold the value of
  // the <surface> tag, but since there is no sdf::Surface and no way to
  // construct an sdf::Collision from a surface element, we are storing the
  // element of the <collision> tag here.
  /// \brief A component that holds the sdf::Element of a collision
  using CollisionElement =
      Component<sdf::ElementPtr, class CollisionElementTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.CollisionElement",
                                CollisionElement)
}
}
}
}

#endif
