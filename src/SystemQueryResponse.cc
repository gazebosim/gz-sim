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

#include "ignition/gazebo/SystemQueryResponse.hh"

using namespace ignition::gazebo;

// Private data class.
class ignition::gazebo::SystemQueryResponsePrivate
{
  /// \brief Constructor
  /// \param[in] _query The entity query.
  /// \param[in] _query The entity component component manager.
  public: SystemQueryResponsePrivate(const EntityQuery &_query,
              EntityComponentManager &_ecMgr)
          : entityQuery(_query), entityComponentMgr(_ecMgr)
  {
  }

  /// \brief The entity query.
  public: const EntityQuery &entityQuery;

  /// \brief The entity component manager.
  public: EntityComponentManager &entityComponentMgr;
};

//////////////////////////////////////////////////
SystemQueryResponse::SystemQueryResponse(const EntityQuery &_query,
    EntityComponentManager &_ecMgr)
  : dataPtr(new SystemQueryResponsePrivate(_query, _ecMgr))
{
}

//////////////////////////////////////////////////
SystemQueryResponse::~SystemQueryResponse()
{
}

//////////////////////////////////////////////////
const EntityQuery &SystemQueryResponse::Query() const
{
  return this->dataPtr->entityQuery;
}

//////////////////////////////////////////////////
EntityComponentManager &SystemQueryResponse::EntityComponentMgr() const
{
  return this->dataPtr->entityComponentMgr;
}
