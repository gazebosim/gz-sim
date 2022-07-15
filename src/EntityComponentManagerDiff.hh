/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GZ_SIM_ENTITYCOMPONENTMANAGER_DIFF_HH_
#define GZ_SIM_ENTITYCOMPONENTMANAGER_DIFF_HH_

#include "gz/sim/Entity.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/Types.hh"

#include <vector>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {

    /// \\brief Used to track the changes in an EntityComponentManager
    ///
    /// Tracks added and removed entities for the purpose of a reset
    class GZ_SIM_VISIBLE EntityComponentManagerDiff
    {
      /// \brief Add an added entity to the current diff
      /// \param[in] _entity Entity that was added
      public: void InsertAddedEntity(const Entity &_entity);

      /// \brief Add a removed entity to the current diff
      /// \param[in] _entity Entity that was removed
      public: void InsertRemovedEntity(const Entity &_entity);

      /// \brief Retrieve the list of added entities
      /// \return Vector of Entity IDs added since construction/clear
      public: const std::vector<Entity> &AddedEntities() const;

      /// \brief Retrieve the list of removed entities
      /// \return Vector of Entity IDs removed since construction/clear
      public: const std::vector<Entity> &RemovedEntities() const;

      /// \brief Clear the list of added entities
      public: void ClearAddedEntities();

      /// \brief Clear the list of removed entities
      public: void ClearRemovedEntities();

      /// \brief List of added entities
      private: std::vector<Entity> addedEntities;

      /// \brief List of removed entities
      private: std::vector<Entity> removedEntities;
    };
  }
  }
}

#endif
