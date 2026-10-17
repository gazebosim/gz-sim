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
#include "JointStateWriter.hh"

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::physics_system
{
//////////////////////////////////////////////////
void WriteJointPositions(EntityComponentManager &_ecm,
    const OwnerFilter &_owner, const JointStateFn &_position)
{
  _ecm.Each<components::Joint, components::JointPosition>(
      [&](const Entity &_entity, components::Joint *,
          components::JointPosition *_jointPos) -> bool
      {
        if (_owner.Owns(_entity) &&
            _position(_entity, _jointPos->Data()))
        {
          _ecm.SetChanged(_entity, components::JointPosition::typeId,
              ComponentState::PeriodicChange);
        }
        return true;
      });
}

//////////////////////////////////////////////////
void WriteJointVelocities(EntityComponentManager &_ecm,
    const OwnerFilter &_owner, const JointStateFn &_velocity)
{
  _ecm.Each<components::Joint, components::JointVelocity>(
      [&](const Entity &_entity, components::Joint *,
          components::JointVelocity *_jointVel) -> bool
      {
        if (_owner.Owns(_entity))
          _velocity(_entity, _jointVel->Data());
        return true;
      });
}
}
}
}
