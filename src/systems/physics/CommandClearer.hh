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
#ifndef GZ_SIM_SYSTEMS_PHYSICS_COMMAND_CLEARER_HH_
#define GZ_SIM_SYSTEMS_PHYSICS_COMMAND_CLEARER_HH_

#include "OwnerFilter.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::physics_system
{
  /// \brief Clear the commands and one-shot requests that a physics provider
  /// has consumed in the current step, for the entities the filter accepts.
  ///
  /// * Removed: JointPositionReset, JointVelocityReset,
  ///   WorldLinearVelocityReset, WorldAngularVelocityReset,
  ///   EnableContactSurfaceCustomization and JointVelocityCmd. Also
  ///   AngularVelocityCmd and LinearVelocityCmd, unless `_zeroVelocityCmds`
  ///   is set.
  /// * Zeroed or emptied, but kept: JointForceCmd, ExternalWorldWrenchCmd,
  ///   JointPositionLimitsCmd, JointVelocityLimitsCmd, JointEffortLimitsCmd
  ///   and SlipComplianceCmd. Also AngularVelocityCmd and LinearVelocityCmd
  ///   if `_zeroVelocityCmds` is set.
  ///
  /// Each provider clears only its own entities. Otherwise a provider that
  /// runs first would erase the commands that another provider has not read
  /// yet.
  ///
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in] _owner Selects the entities to clear.
  /// \param[in] _zeroVelocityCmds True to set the linear and angular velocity
  /// commands to zero instead of removing them. A system such as
  /// VelocityControl writes them every step. When the component is gone it
  /// creates it again, which costs several times more than overwriting it.
  /// A provider that treats a zero command as no motion can use this.
  void ClearCommands(EntityComponentManager &_ecm, const OwnerFilter &_owner,
      bool _zeroVelocityCmds = false);
}
}
}

#endif
