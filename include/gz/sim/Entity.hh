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
#ifndef GZ_SIM_ENTITY_HH_
#define GZ_SIM_ENTITY_HH_

#include <cstdint>
#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

/// \brief This library is part of the [Gazebo](https://gazebosim.org) project.
namespace gz
{
  /// \brief Gazebo is a leading open source robotics simulator, that
  /// provides high fidelity physics, rendering, and sensor simulation.
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Adding component namespace information here because there is
    // currently no one component class that seems like a good place to hold
    // this documentation.
    /// \brief Components represent data, such as position information. An
    /// Entity usually has one or more associated components.
    ///
    /// The set of Components assigned to an Entity also act as a key.
    /// Systems process Entities based on their key. For example, a physics
    /// system may process only entities that have pose and inertia
    /// components.
    namespace components {}

    /// \brief An Entity identifies a single object in simulation such as
    /// a model, link, or light. At its core, an Entity is just an identifier.
    ///
    /// An Entity usually has one or more associated Components. Components
    /// represent data, such as position information.
    ///
    /// The set of Components assigned to an Entity also act as a key.
    /// Systems process Entities based on their key. For example, a physics
    /// system may process only entities that have pose and inertia
    /// components.
    ///
    /// An Entity that needs to be identified and used by Systems should be
    /// created through the EntityComponentManager.
    using Entity = uint64_t;

    /// \brief Indicates a non-existant or invalid Entity.
    const Entity kNullEntity{0};
    }
  }
}
#endif
