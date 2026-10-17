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
#include "CommandClearer.hh"

#include <algorithm>
#include <vector>

#include <gz/math/Vector3.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/AngularVelocityReset.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/JointEffortLimitsCmd.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPositionLimitsCmd.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointVelocityLimitsCmd.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityReset.hh"
#include "gz/sim/components/SlipComplianceCmd.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::physics_system
{
namespace
{
/// \brief Remove a component from every entity that the filter accepts.
/// \tparam ComponentT Component type.
/// \param[in,out] _ecm Mutable reference to the ECM.
/// \param[in] _owner Selects the entities to clear.
template<typename ComponentT>
void RemoveComponents(EntityComponentManager &_ecm, const OwnerFilter &_owner)
{
  // Components cannot be removed while iterating over them.
  std::vector<Entity> entities;
  _ecm.Each<ComponentT>(
      [&](const Entity &_entity, ComponentT *) -> bool
      {
        if (_owner.Owns(_entity))
          entities.push_back(_entity);
        return true;
      });

  for (const auto entity : entities)
    _ecm.RemoveComponent<ComponentT>(entity);
}

/// \brief Reset the data of a component on every entity that the filter
/// accepts. The component stays on the entity.
/// \tparam ComponentT Component type.
/// \tparam ClearFn Callable that resets the data of one component.
/// \param[in,out] _ecm Mutable reference to the ECM.
/// \param[in] _owner Selects the entities to clear.
/// \param[in] _clear Resets the data of one component.
template<typename ComponentT, typename ClearFn>
void ResetComponents(EntityComponentManager &_ecm, const OwnerFilter &_owner,
    const ClearFn &_clear)
{
  _ecm.Each<ComponentT>(
      [&](const Entity &_entity, ComponentT *_comp) -> bool
      {
        if (_owner.Owns(_entity))
          _clear(_comp->Data());
        return true;
      });
}
}

//////////////////////////////////////////////////
void ClearCommands(EntityComponentManager &_ecm, const OwnerFilter &_owner,
    bool _zeroVelocityCmds)
{
  // Clear reset components
  RemoveComponents<components::JointPositionReset>(_ecm, _owner);
  RemoveComponents<components::JointVelocityReset>(_ecm, _owner);
  RemoveComponents<components::WorldLinearVelocityReset>(_ecm, _owner);
  RemoveComponents<components::WorldAngularVelocityReset>(_ecm, _owner);
  RemoveComponents<components::EnableContactSurfaceCustomization>(_ecm, _owner);

  // Clear pending commands
  ResetComponents<components::JointForceCmd>(_ecm, _owner,
      [](auto &_data) {std::fill(_data.begin(), _data.end(), 0.0);});
  ResetComponents<components::ExternalWorldWrenchCmd>(_ecm, _owner,
      [](auto &_data) {_data.Clear();});
  ResetComponents<components::JointPositionLimitsCmd>(_ecm, _owner,
      [](auto &_data) {_data.clear();});
  ResetComponents<components::JointVelocityLimitsCmd>(_ecm, _owner,
      [](auto &_data) {_data.clear();});
  ResetComponents<components::JointEffortLimitsCmd>(_ecm, _owner,
      [](auto &_data) {_data.clear();});
  RemoveComponents<components::JointVelocityCmd>(_ecm, _owner);
  ResetComponents<components::SlipComplianceCmd>(_ecm, _owner,
      [](auto &_data) {std::fill(_data.begin(), _data.end(), 0.0);});
  if (_zeroVelocityCmds)
  {
    ResetComponents<components::AngularVelocityCmd>(_ecm, _owner,
        [](auto &_data) {_data = math::Vector3d::Zero;});
    ResetComponents<components::LinearVelocityCmd>(_ecm, _owner,
        [](auto &_data) {_data = math::Vector3d::Zero;});
  }
  else
  {
    RemoveComponents<components::AngularVelocityCmd>(_ecm, _owner);
    RemoveComponents<components::LinearVelocityCmd>(_ecm, _owner);
  }
}
}
}
}
