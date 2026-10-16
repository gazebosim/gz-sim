/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_PHYSICSOWNER_HH_
#define GZ_SIM_COMPONENTS_PHYSICSOWNER_HH_

#include <string>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief The system that owns the physics of a model, or of every model in
  /// a world.
  ///
  /// The data is the name of the owning physics provider. By convention this
  /// is the `name` attribute of the provider's `<plugin>` element, for example
  /// `gz::sim::systems::SimplifiedPhysics`. Matching is an exact string
  /// compare, so every user of the component must spell the name the same
  /// way. The empty string selects the default provider, which owns every
  /// entity that has no PhysicsOwner anywhere above it.
  ///
  /// The component can sit on a top-level model or on the world:
  ///
  /// * On a top-level model, it applies to that model and all of its
  ///   descendants.
  /// * On the world, it is the default for every model that does not carry its
  ///   own PhysicsOwner. A model's component, even an empty one, takes
  ///   precedence over the world's.
  ///
  /// The component is only a declaration. Each physics provider decides which
  /// entities to simulate, using `gz::sim::isOwnedBy` from `gz/sim/Util.hh`.
  /// If the component is absent everywhere, the default provider owns the
  /// entity.
  using PhysicsOwner = Component<std::string, class PhysicsOwnerTag,
      serializers::StringSerializer>;
  GZ_SIM_DECLARE_COMPONENT("gz_sim_components.PhysicsOwner", PhysicsOwner)
}
}
}
}

#endif
