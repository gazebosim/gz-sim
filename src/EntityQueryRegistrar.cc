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

#include <utility>
#include <vector>

#include "ignition/gazebo/EntityQueryRegistrar.hh"

using namespace ignition::gazebo;

class ignition::gazebo::EntityQueryRegistrarPrivate
{
  /// \brief queries and callbacks that have been registered
  public: std::vector<EntityQueryRegistration> queryCallbacks;
};

/////////////////////////////////////////////////
EntityQueryRegistrar::EntityQueryRegistrar() :
  dataPtr(new EntityQueryRegistrarPrivate)
{
}

/////////////////////////////////////////////////
EntityQueryRegistrar::~EntityQueryRegistrar()
{
}

/////////////////////////////////////////////////
void EntityQueryRegistrar::Register(const EntityQuery &_q,
    EntityQueryCallback _cb)
{
  this->dataPtr->queryCallbacks.push_back({_q, _cb});
}

/////////////////////////////////////////////////
std::vector<EntityQueryRegistration> EntityQueryRegistrar::Registrations() const
{
  return this->dataPtr->queryCallbacks;
}

