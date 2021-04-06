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
#include "ignition/gazebo/detail/View.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace ignition;
using namespace gazebo;
using namespace detail;

//////////////////////////////////////////////////
void View::AddEntity(const Entity _entity, const bool _new)
{
  this->entities.insert(_entity);
  if (_new)
  {
    this->newEntities.insert(_entity);
  }
}

//////////////////////////////////////////////////
void View::AddComponent(const Entity _entity,
    const ComponentTypeId _typeId,
    const ComponentId _componentId)
{
  this->components.insert(
      std::make_pair(std::make_pair(_entity, _typeId), _componentId));
}

//////////////////////////////////////////////////
bool View::RemoveEntity(const Entity _entity, const ComponentTypeKey &_key)
{
  if (this->entities.find(_entity) == this->entities.end())
    return false;

  // Otherwise, remove the entity from the view
  this->entities.erase(_entity);
  this->newEntities.erase(_entity);
  this->toRemoveEntities.erase(_entity);

  // Remove the entity from the components map
  for (const ComponentTypeId &compTypeId : _key)
    this->components.erase(std::make_pair(_entity, compTypeId));

  return true;
}

/////////////////////////////////////////////////
const components::BaseComponent *View::ComponentImplementation(
    const Entity _entity,
    ComponentTypeId _typeId,
    const EntityComponentManager *_ecm) const
{
  return _ecm->ComponentImplementation(
      {_typeId, this->components.at({_entity, _typeId})});
}

//////////////////////////////////////////////////
void View::ClearNewEntities()
{
  this->newEntities.clear();
}

//////////////////////////////////////////////////
bool View::AddEntityToRemoved(const Entity _entity)
{
  if (this->entities.find(_entity) == this->entities.end())
    return false;
  this->toRemoveEntities.insert(_entity);
  return true;
}
