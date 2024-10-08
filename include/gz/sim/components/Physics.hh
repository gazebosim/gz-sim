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
#ifndef GZ_SIM_COMPONENTS_PHYSICS_HH_
#define GZ_SIM_COMPONENTS_PHYSICS_HH_

#include <cstdint>
#include <string>

#include <gz/msgs/physics.pb.h>

#include <sdf/Physics.hh>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

#include <gz/sim/components/Factory.hh>
#include "gz/sim/components/Component.hh"
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/Conversions.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
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
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.Physics",
      Physics)

  /// \brief The name of the collision detector to be used. The supported
  /// options will depend on the physics engine being used.
  using PhysicsCollisionDetector = Component<std::string,
      class PhysicsCollisionDetectorTag, serializers::StringSerializer>;
  GZ_SIM_REGISTER_COMPONENT(
      "gz_sim_components.PhysicsCollisionDetector",
       PhysicsCollisionDetector)

  /// \brief The name of the solver to be used. The supported options will
  /// depend on the physics engine being used.
  using PhysicsSolver = Component<std::string,
      class PhysicsSolverTag, serializers::StringSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.PhysicsSolver",
       PhysicsSolver)

  /// \brief The number of solver iterations for each step.
  using PhysicsSolverIterations = Component<uint32_t,
      class PhysicsSolverIterationsTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.PhysicsSolverIterations",
       PhysicsSolverIterations)
}
}
}
}

#endif
