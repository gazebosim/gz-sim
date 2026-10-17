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
#ifndef GZ_SIM_SYSTEMS_PHYSICS_JOINT_STATE_WRITER_HH_
#define GZ_SIM_SYSTEMS_PHYSICS_JOINT_STATE_WRITER_HH_

#include <functional>
#include <vector>

#include "OwnerFilter.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::physics_system
{
  /// \brief Reads the state of one joint from the simulation.
  /// \param[in] _joint Joint entity.
  /// \param[out] _state One value for each degree of freedom of the joint.
  /// The vector holds the previous values on entry, so a caller can resize
  /// it in place.
  /// \return True if the caller simulates the joint and filled `_state`. If
  /// false, the joint's component is left untouched.
  using JointStateFn = std::function<bool(const Entity &_joint,
      std::vector<double> &_state)>;

  /// \brief Write the JointPosition component of every joint that has one and
  /// that the filter accepts. The component is marked as periodically
  /// changed.
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in] _owner Selects the joints to write.
  /// \param[in] _position Returns the joint positions.
  void WriteJointPositions(EntityComponentManager &_ecm,
      const OwnerFilter &_owner, const JointStateFn &_position);

  /// \brief Write the JointVelocity component of every joint that has one
  /// and that the filter accepts. The component is written but not marked as
  /// changed, so state messages do not carry a new velocity by itself.
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in] _owner Selects the joints to write.
  /// \param[in] _velocity Returns the joint velocities.
  void WriteJointVelocities(EntityComponentManager &_ecm,
      const OwnerFilter &_owner, const JointStateFn &_velocity);
}
}
}

#endif
