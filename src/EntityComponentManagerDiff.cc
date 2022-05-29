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

#include "EntityComponentManagerDiff.hh"

#include "gz/sim/Entity.hh"

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
void EntityComponentManagerDiff::InsertAddedEntity(const Entity &_entity)
{
  this->addedEntities.push_back(_entity);
}

//////////////////////////////////////////////////
void EntityComponentManagerDiff::InsertRemovedEntity(const Entity &_entity)
{
  this->removedEntities.push_back(_entity);
}

//////////////////////////////////////////////////
const std::vector<Entity> &EntityComponentManagerDiff::AddedEntities() const
{
  return this->addedEntities;
}

//////////////////////////////////////////////////
const std::vector<Entity> &EntityComponentManagerDiff::RemovedEntities() const
{
  return this->removedEntities;
}

//////////////////////////////////////////////////
void EntityComponentManagerDiff::ClearAddedEntities()
{
  this->addedEntities.clear();
}

//////////////////////////////////////////////////
void EntityComponentManagerDiff::ClearRemovedEntities()
{
  this->removedEntities.clear();
}
