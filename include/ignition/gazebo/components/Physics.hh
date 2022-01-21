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
#ifndef IGNITION_GAZEBO_COMPONENTS_PHYSICS_HH_
#define IGNITION_GAZEBO_COMPONENTS_PHYSICS_HH_

#include <string>

#include <ignition/msgs/physics.pb.h>

#include <sdf/Physics.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include <ignition/gazebo/components/Factory.hh>
#include "ignition/gazebo/components/Component.hh"
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/Conversions.hh>

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

  /// \brief The name of the collision detector to be used. The supported
  /// options will depend on the physics engine being used.
  using PhysicsCollisionDetector = Component<std::string,
      class PhysicsCollisionDetectorTag, serializers::StringSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.PhysicsCollisionDetector",
       PhysicsCollisionDetector)

  /// \brief The name of the solver to be used. The supported options will
  /// depend on the physics engine being used.
  using PhysicsSolver = Component<std::string,
      class PhysicsSolverTag, serializers::StringSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.PhysicsSolver",
       PhysicsSolver)

  /// \brief A component used to indicate the active physics entity.
  using ActivePhysicsEntity = Component<Entity, class ActivePhysicsEntiyTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.ActivePhysicsEntity",
      ActivePhysicsEntity)
}
}
}
}

#endif
