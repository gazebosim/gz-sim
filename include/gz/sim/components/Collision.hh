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
#ifndef GZ_SIM_COMPONENTS_COLLISION_HH_
#define GZ_SIM_COMPONENTS_COLLISION_HH_

#include <sdf/Element.hh>

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
namespace serializers
{
  using CollisionElementSerializer =
      serializers::ComponentToMsgSerializer<sdf::Collision, msgs::Collision>;
}

namespace components
{
  /// \brief A component that identifies an entity as being a collision.
  using Collision = Component<NoData, class CollisionTag>;
  GZ_SIM_REGISTER_COMPONENT(
      "gz_sim_components.Collision", Collision)

  // TODO(anyone) The sdf::Collision DOM object does not yet contain
  // surface information.
  /// \brief A component that holds the sdf::Collision object.
  using CollisionElement =
      Component<sdf::Collision, class CollisionElementTag,
    serializers::CollisionElementSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CollisionElement",
                                CollisionElement)

  /// \brief A component used to enable customization of contact surface for a
  /// collision. The customization itself is done in callback of event
  /// CollectContactSurfaceProperties from PhysicsEvents.
  using EnableContactSurfaceCustomization =
    Component<bool, class EnableContactSurfaceCustomizationTag>;
  GZ_SIM_REGISTER_COMPONENT(
    "gz_sim_components.EnableContactSurfaceCustomization",
    EnableContactSurfaceCustomization)
}
}
}
}

#endif
