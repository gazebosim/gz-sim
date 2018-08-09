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
#ifndef IGNITION_GAZEBO_SYSTEM_QUERY_RESPONSE_HH_
#define IGNITION_GAZEBO_SYSTEM_QUERY_RESPONSE_HH_

#include <memory>
#include <string>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityQuery.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class SystemQueryResponsePrivate;

    /** \class SystemQueryResponse SystemQueryResponse.hh \
     * ignition/gazebo/SystemQueryResponse.hh
    **/
    /// \brief Base class for a System.
    ///
    /// A System operates on Entities that have certain Components. A System
    /// will only operate on an Entity if it has all of the required
    /// Components.
    class IGNITION_GAZEBO_VISIBLE SystemQueryResponse
    {
      /// \brief Constructor
      /// \param[in] _query The entity query.
      /// \param[in] _query The entity component component manager.
      public: SystemQueryResponse(const EntityQuery &_query,
                  EntityComponentManager &_ecMgr);

      /// \brief Destructor
      public: ~SystemQueryResponse();

      /// \brief Get the EntityQuery.
      /// \return Reference to the entity query associated with this
      /// response.
      public: const EntityQuery &Query() const;

      /// \brief Get the EntityComponentManager, which can be used to access
      /// entities and their components.
      /// \return A reference to the EntityComponentManager.
      public: EntityComponentManager &EntityComponentMgr() const;

      /// \brief Private data pointer.
      private: std::unique_ptr<SystemQueryResponsePrivate> dataPtr;
    };
    }
  }
}
#endif
