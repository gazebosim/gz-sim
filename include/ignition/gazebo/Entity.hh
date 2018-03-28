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
#ifndef IGNITION_GAZEBO_ENTITY_HH_
#define IGNITION_GAZEBO_ENTITY_HH_

namespace ignition
{
  namespace gazebo
  {
    /// \brief An Entity is an id!
    using EntityId = int;

    /// \brief For results which there is no entity
    const EntityId kNullEntity = -1;

    // Forward Declaration
    class EntityPrivate;

    /// \brief An Entity identifies a single object in simulation such as
    /// a model, link, or light. At its core, an Entity is just and identifier.
    ///
    /// An Entity usually has one or more associated Components. Components
    /// represent data, such as position information.
    ///
    /// The set of Components assigned to an Entity also act as a key.
    /// Systems process Entities based on their key. For example, a physics
    /// system may process only entities that have pose and inertia
    /// components.
    class Entity
    {
      /// \brief Equality operator. Checks if this Entity is equivalent to
      /// the provided Entity.
      /// \param[in] _entity Entity to compare.
      /// \returns true if the Entity Id's are the same.
      public: bool operator==(const Entity &_entity) const;

      /// \brief Return id of entity.
      /// \return Id of this Entity.
      public: EntityId Id() const;

      // Note: We are not using the private data pattern on purpose. An Entity
      // should only have an Id.

      /// \brief Id of the entity
      private: EntityId id = kNullEntity;
    };
  }
}
#endif
