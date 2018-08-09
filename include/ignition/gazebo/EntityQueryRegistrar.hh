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

#ifndef IGNITION_GAZEBO_ENTITY_QUERY_REGISTRAR_HH_
#define IGNITION_GAZEBO_ENTITY_QUERY_REGISTRAR_HH_

#include <memory>
#include <vector>

#include <ignition/gazebo/EntityQuery.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations
    class EntityQueryRegistrarPrivate;

    /// \brief Registers callbacks for Entity query results
    class IGNITION_GAZEBO_VISIBLE EntityQueryRegistrar
    {
      /// \brief Default constructor.
      public: EntityQueryRegistrar();

      /// \brief Destructor.
      public: ~EntityQueryRegistrar();

      /// \brief Adds a query and a callback to call with the results.
      /// \param[in] _q the query object.
      /// \param[in] _cb callback function.
      public: void Register(const EntityQuery &_q, EntityQueryCallback _cb);

      /// \brief Return the registered callbacks
      public: std::vector<EntityQueryRegistration> Registrations() const;

      /// \brief Private data pointer
      private: std::unique_ptr<EntityQueryRegistrarPrivate> dataPtr;
    };
    }
  }
}

#endif
