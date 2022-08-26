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
#ifndef GZ_GAZEBO_COMPONENTS_PHYSICS_HH_
#define GZ_GAZEBO_COMPONENTS_PHYSICS_HH_

#include <ignition/msgs/physics.pb.h>

#include <sdf/Physics.hh>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

#include <gz/sim/components/Factory.hh>
#include "gz/sim/components/Component.hh"
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/Conversions.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace serializers
{
  using PhysicsSerializer =
      serializers::ComponentToMsgSerializer<sdf::Physics, msgs::Physics>;
}
namespace components
{
  /// \brief A component type that contains the physics properties of
  /// the World entity.
  using Physics = Component<sdf::Physics, class PhysicsTag,
      serializers::PhysicsSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.Physics",
      Physics)
}
}
}
}

#endif
