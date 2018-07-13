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
#include <ignition/gazebo/ComponentType.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    /// \brief Forward declaration
    class EntityQueryPrivate;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    /// \class EntityQuery EntityQuery.hh ignition/gazebo/EntityQuery.hh
    /// \brief a Class for querying entities from a manager
    class IGNITION_GAZEBO_VISIBLE EntityQuery
    {
      /// \brief Constructor
      public: EntityQuery();

      /// \brief Destructor
      public: ~EntityQuery();

      /// \brief Return true if this is an empty query.
      public: bool Empty() const;

      public: bool AddComponentType(const ComponentType &_type);

      /// \brief Add a component based on a component type.
      /// \param[in] _type Type of component to add.
      /// \return True if the _type was successfully added.
      public: bool AddComponentType(const ComponentTypeId _type);

      /// \brief Get the components that have been added to the query.
      /// \return A const reference to the set of components in this query.
      public: const std::set<ComponentTypeId> &ComponentTypes() const;

      /// \brief Returns true if these are the same queries.
      /// \param[in] _query The query to compare
      /// \return True if this query matches _query.
      public: bool operator==(const EntityQuery &_query) const;

      /// \brief Private data pointer
      private: std::unique_ptr<EntityQueryPrivate> dataPtr;
    };
    }
  }
}
#endif
