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
#ifndef GZ_SIM_COMPONENTS_PARENTENTITY_HH_
#define GZ_SIM_COMPONENTS_PARENTENTITY_HH_

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Entity.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief This component holds an entity's parent entity.
  ///
  /// Note that the EntityComponentManager also keeps track of each entity's
  /// children, and that information is kept in sync when `ParentEntity`
  /// components are created, removed or set through the
  /// EntityComponentManager (i.e. `CreateComponent`, `RemoveComponent`,
  /// `SetComponentData`, `SetParentEntity` and `SetState`).
  /// Modifying the data of an existing `ParentEntity` component directly,
  /// e.g. through `Component<ParentEntity>(entity)->Data()` or
  /// `Component<ParentEntity>(entity)->SetData()`, bypasses this and will
  /// leave the hierarchy out of sync. Use
  /// `EntityComponentManager::SetParentEntity` instead.
  using ParentEntity = Component<Entity, class ParentEntityTag>;
  GZ_SIM_DECLARE_COMPONENT(
      "gz_sim_components.ParentEntity", ParentEntity)
}
}
}
}

#endif
