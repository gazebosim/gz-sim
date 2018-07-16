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

#ifndef IGNITION_GAZEBO_ENTITY_QUERY_RESULT_HH_
#define IGNITION_GAZEBO_ENTITY_QUERY_RESULT_HH_

#include <memory>
#include <ignition/gazebo/EntityQuery.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    /// \brief forward declaration
    class EntityQueryResultPrivate;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    /// \brief Registers callbacks for Entity query results
    class IGNITION_GAZEBO_VISIBLE EntityQueryResult
    {
      /// \brief Default constructor.
      public: EntityQueryResult() = default;

      /// \brief Destructor.
      public: ~EntityQueryResult() = default;

      /// \brief Private data pointer
      // private: std::unique_ptr<EntityQueryResult> dataPtr;
    };
    }
  }
}

#endif
