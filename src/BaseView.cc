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
#include "gz/sim/detail/BaseView.hh"

#include "gz/sim/Entity.hh"
#include "gz/sim/Types.hh"

using namespace gz;
using namespace sim;
using namespace detail;

//////////////////////////////////////////////////
BaseView::~BaseView() = default;

//////////////////////////////////////////////////
bool BaseView::HasEntity(const Entity _entity) const
{
  return this->entities.find(_entity) != this->entities.end();
}

//////////////////////////////////////////////////
bool BaseView::IsEntityMarkedForAddition(const Entity _entity) const
{
  return this->toAddEntities.find(_entity) != this->toAddEntities.end();
}

//////////////////////////////////////////////////
bool BaseView::MarkEntityToAdd(const Entity _entity, bool _new)
{
  if (this->HasCachedComponentData(_entity))
    return false;

  this->toAddEntities[_entity] = _new;
  return true;
}

//////////////////////////////////////////////////
bool BaseView::RequiresComponent(const ComponentTypeId _typeId) const
{
  return this->componentTypes.find(_typeId) != this->componentTypes.end();
}

//////////////////////////////////////////////////
bool BaseView::MarkEntityToRemove(const Entity _entity)
{
  if (this->HasCachedComponentData(_entity) ||
      this->IsEntityMarkedForAddition(_entity))
  {
    this->toRemoveEntities.insert(_entity);
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
void BaseView::ResetNewEntityState()
{
  this->newEntities.clear();

  // mark all entities in the toAddEntities map as not newly created
  for (auto &entityNewPair : this->toAddEntities)
    entityNewPair.second = false;
}

//////////////////////////////////////////////////
const std::set<ComponentTypeId> &BaseView::ComponentTypes() const
{
  return this->componentTypes;
}

const std::set<Entity> &BaseView::Entities() const
{
  return this->entities;
}

//////////////////////////////////////////////////
const std::set<Entity> &BaseView::NewEntities() const
{
  return this->newEntities;
}

//////////////////////////////////////////////////
const std::set<Entity> &BaseView::ToRemoveEntities() const
{
  return this->toRemoveEntities;
}

//////////////////////////////////////////////////
const std::unordered_map<Entity, bool> &BaseView::ToAddEntities() const
{
  return this->toAddEntities;
}

//////////////////////////////////////////////////
void BaseView::ClearToAddEntities()
{
  this->toAddEntities.clear();
}
