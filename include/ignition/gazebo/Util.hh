/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_UTIL_HH_
#define IGNITION_GAZEBO_UTIL_HH_

#include <string>
#include <unordered_set>

#include <ignition/math/Pose3.hh>
#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    /// \brief Helper function to compute world pose of an entity
    /// \param[in] _entity Entity to get the world pose for
    /// \param[in] _ecm Immutable reference to ECM.
    /// \return World pose of entity
    math::Pose3d IGNITION_GAZEBO_VISIBLE worldPose(const Entity &_entity,
        const EntityComponentManager &_ecm);

    /// \brief Helper function to generate scoped name for an entity.
    /// \param[in] _entity Entity to get the name for.
    /// \param[in] _ecm Immutable reference to ECM.
    /// \param[in] _delim Delimiter to put between names, defaults to "/".
    /// \param[in] _includePrefix True to include the type prefix before the
    /// entity name
    std::string IGNITION_GAZEBO_VISIBLE scopedName(const Entity &_entity,
      const EntityComponentManager &_ecm, const std::string &_delim = "/",
      bool _includePrefix = true);

    /// \brief Get all entities which are descendants of a given entity,
    /// including the entity itself.
    /// \param[in] _entity Entity whose descendants we want.
    /// \param[in] _ecm Immutable reference to ECM.
    /// \return All child entities recursively, including _entity.
    std::unordered_set<Entity> IGNITION_GAZEBO_VISIBLE
        descendants(Entity _entity, const EntityComponentManager &_ecm);
    }
  }
}
#endif
