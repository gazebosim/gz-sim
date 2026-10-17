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

#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/components/PhysicsOwner.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
/// \brief Test system that gives its entity a PhysicsOwner in Configure, as a
/// physics provider's model plugin would. It lets a test check that the
/// component exists before the Physics system sees a model that is spawned
/// while the simulation runs.
///
/// Parameters:
///   * `<owner>`: name of the owning provider. Defaults to the empty string.
class SetPhysicsOwnerSystem :
  public System,
  public ISystemConfigure
{
  // Documentation inherited
  public: void Configure(const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &/*_eventMgr*/) override
  {
    const auto owner = _sdf->Get<std::string>("owner", "").first;
    _ecm.CreateComponent(_entity, components::PhysicsOwner(owner));
  }
};
}
}
}

GZ_ADD_PLUGIN(gz::sim::SetPhysicsOwnerSystem,
    gz::sim::System,
    gz::sim::SetPhysicsOwnerSystem::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(gz::sim::SetPhysicsOwnerSystem,
    "gz::sim::SetPhysicsOwnerSystem")
