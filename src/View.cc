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
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
void View::AddEntity(const EntityId _id, const bool _new)
{
  this->entities.insert(_id);
  if (_new)
  {
    this->newEntities.insert(_id);
  }
}

//////////////////////////////////////////////////
void View::AddComponent(const EntityId _id,
    const ComponentTypeId _compId,
    const ComponentId _componentId)
{
  this->components.insert(
      std::make_pair(std::make_pair(_id, _compId), _componentId));
}

//////////////////////////////////////////////////
bool View::EraseEntity(const EntityId _id, const ComponentTypeKey &_key)
{
  if (this->entities.find(_id) == this->entities.end())
    return false;

  // Otherwise, remove the entity from the view
  this->entities.erase(_id);
  this->newEntities.erase(_id);
  this->toEraseEntities.erase(_id);

  // Remove the entity from the components map
  for (const ComponentTypeId &compTypeId : _key)
    this->components.erase(std::make_pair(_id, compTypeId));

  return true;
}

/////////////////////////////////////////////////
const void *View::ComponentImplementation(const EntityId _id,
    ComponentTypeId _typeId,
    const EntityComponentManager *_ecm) const
{
  return _ecm->ComponentImplementation(
      {_typeId, this->components.at({_id, _typeId})});
}

//////////////////////////////////////////////////
void View::ClearNewEntities()
{
  this->newEntities.clear();
}

//////////////////////////////////////////////////
bool View::AddEntityToErased(const EntityId _id)
{
  if (this->entities.find(_id) == this->entities.end())
    return false;
  this->toEraseEntities.insert(_id);
  return true;
}
