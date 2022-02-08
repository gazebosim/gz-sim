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

#ifndef IGNITION_GAZEBO_ENTITYCOMPONENTMANAGER_DIFF_HH_
#define IGNITION_GAZEBO_ENTITYCOMPONENTMANAGER_DIFF_HH_

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

#include <vector>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

    class EntityComponentManagerDiff
    {
      public: void InsertAddedEntity(const Entity &_entity);
      public: void InsertRemovedEntity(const Entity &_entity);

      public: const std::vector<Entity> &RemovedEntities() const;
      public: const std::vector<Entity> &AddedEntities() const;

      public: void ClearAddedEntities();
      public: void ClearRemovedEntities();

      private: std::vector<Entity> addedEntities;
      private: std::vector<Entity> removedEntities;
    };
  }
  }
}

#endif
