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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_PHYSICS_MODE_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_PHYSICS_MODE_HH_

#include <string>
#include <vector>

#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
  /// \brief Name of the SimplifiedPhysics system as it is stored in
  /// components::PhysicsOwner. It is the `name` attribute of the system's
  /// `<plugin>` element.
  constexpr const char *kProviderName{"gz::sim::systems::SimplifiedPhysics"};

  /// \brief How the physics of one model is simulated.
  enum class PhysicsMode
  {
    /// \brief The model is left to the default physics system.
    Full,

    /// \brief SimplifiedPhysics moves the model in x, y and yaw at the
    /// commanded velocity.
    PlanarKinematic,

    /// \brief Planned. Parsed so that a clear message can be printed, but
    /// SimplifiedPhysics does not simulate it yet.
    PlanarDynamic,
  };

  /// \brief Parse the value of a `<mode>` element.
  /// \param[in] _name One of `full`, `planar_kinematic` or `planar_dynamic`.
  /// \param[out] _mode The mode, only set if the name is known.
  /// \return True if the name is known.
  bool ParseMode(const std::string &_name, PhysicsMode &_mode);

  /// \brief The name of a mode, as ParseMode accepts it.
  /// \param[in] _mode A mode.
  /// \return The name.
  std::string ModeName(PhysicsMode _mode);

  /// \brief The physics features that SimplifiedPhysics serves for a mode.
  ///
  /// The names are the ones that systems list in their physics requirements
  /// (see ISystemPhysicsRequirements). The list is written to
  /// components::SupportedPhysicsFeatures, which is what the system manager
  /// checks. Nothing checks the mode name.
  ///
  /// \param[in] _mode A mode.
  /// \return The features. No planar mode serves any feature yet.
  std::vector<std::string> SupportedFeatures(PhysicsMode _mode);
}
}
}

#endif
