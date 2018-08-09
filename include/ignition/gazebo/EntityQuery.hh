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

#ifndef IGNITION_GAZEBO_ENTITY_QUERY_HH_
#define IGNITION_GAZEBO_ENTITY_QUERY_HH_

#include <memory>
#include <set>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations
    class EntityQueryPrivate;

    /// \class EntityQuery EntityQuery.hh ignition/gazebo/EntityQuery.hh
    /// \brief A class for querying entities from a manager
    /// (louise) Is it used for both querying and responding? Maybe it makes
    /// sense to decouple these?
    class IGNITION_GAZEBO_VISIBLE EntityQuery
    {
      /// \brief Constructor.
      public: EntityQuery();

      /// \brief Copy constructor.
      /// \param[in] _query Query to copy.
      public: EntityQuery(const EntityQuery &_query);

      /// \brief Move constructor.
      /// \param[in] _query Query to move.
      public: EntityQuery(EntityQuery &&_query) noexcept;

      /// \brief Destructor.
      public: ~EntityQuery();

      /// \brief Return true if this is an empty query.
      public: bool Empty() const;

      /// \brief Add a component based on a component type.
      /// \param[in] _type Type of component to add.
      /// \return True if the _type was successfully added.
      public: bool AddComponentType(const ComponentTypeId _type);

      /// \brief Get the components that have been added to the query.
      /// \return A const reference to the set of components in this query.
      public: const std::set<ComponentTypeId> &ComponentTypes() const;

      /// \brief Add an entity to the query result.
      /// \param[in] _id Id of the entity to add.
      /// \return True if the entity was added.
      public: bool AddEntity(const EntityId _id);

      /// \brief Remove an entity from the query result.
      /// \param[in] _id Id of the entity
      /// \return True if the entity was removed.
      public: void RemoveEntity(const EntityId _id);

      /// \brief Get the entity ids that match this query.
      /// \todo ordered results matching component placement in memory
      /// \return The entities that match the components in this query.
      /// \sa AddEntity
      /// \sa RemoveEntity
      public: const std::set<EntityId> &Entities() const;

      /// \brief Get the number of entities in this query's result.
      /// \return Count of the entities that match the specified components.
      public: size_t EntityCount() const;

      /// \brief Clear results of a query. This will keep the set of
      /// components, and clear the set of entities.
      public: void Clear();

      /// \brief Returns true if these are the same queries. This will
      /// return true only if the ComponentTypes match.
      /// \param[in] _query The query to compare
      /// \return True if this query matches _query.
      public: bool operator==(const EntityQuery &_query) const;

      /// \brief Returns true if these are not the same queries. This will
      /// return true only if the ComponentTypes do not match.
      /// \param[in] _query The query to compare
      /// \return True if this query does not match _query.
      public: bool operator!=(const EntityQuery &_query) const;

      /// \brief Assignment operator.
      /// \param[in] _query Query to copy.
      /// \return Reference to this object.
      public: EntityQuery &operator=(const EntityQuery &_query);

      /// \brief Move assignment operator.
      /// \param[in] _query Query to copy.
      /// \return Reference to this object.
      public: EntityQuery &operator=(EntityQuery &&_query);

      /// \brief Private data pointer
      private: std::unique_ptr<EntityQueryPrivate> dataPtr;
    };
    }
  }
}
#endif
