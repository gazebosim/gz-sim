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
#ifndef IGNITION_GAZEBO_COMPONENTS_WORLD_HH_
#define IGNITION_GAZEBO_COMPONENTS_WORLD_HH_

#include <sdf/World.hh>

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
  /// \brief A component that identifies an entity as being a world.
  using World = Component<NoData, class WorldTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.World", World)

  /// \brief A component that holds the world's SDF DOM
  using WorldSdf = Component<sdf::World, class WorldTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.WorldSdf", WorldSdf)
}
}
}
}

#endif
