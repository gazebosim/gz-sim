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
#include "ignition/gazebo/ComponentManager.hh"

#include <map>
#include <set>
#include <vector>

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::ComponentManagerPrivate
{
  public: bool EntityMatches(EntityId _id,
    const std::set<ComponentTypeId> &_types) const;

  /// \brief instances of entities
  public: std::vector<Entity> entities;

  /// \brief deleted entity ids that can be reused
  public: std::set<EntityId> availableEntityIds;

  public: std::map<EntityId, std::vector<ComponentKey>> entityComponents;

  /// \brief Set of known Entity queries.
  public: std::vector<EntityQuery> queries;
};

//////////////////////////////////////////////////
ComponentManager::ComponentManager()
  : dataPtr(new ComponentManagerPrivate)
{
}

//////////////////////////////////////////////////
ComponentManager::~ComponentManager()
{
}

//////////////////////////////////////////////////
size_t ComponentManager::EntityCount() const
{
  return this->dataPtr->entities.size();
}

/////////////////////////////////////////////////
EntityId ComponentManager::CreateEntity()
{
  EntityId id = kNullEntity;

  if (!this->dataPtr->availableEntityIds.empty())
  {
    // Reuse the smallest available EntityId
    id = *(this->dataPtr->availableEntityIds.begin());
    this->dataPtr->availableEntityIds.erase(
        this->dataPtr->availableEntityIds.begin());
    this->dataPtr->entities[id] = std::move(Entity(id));
  }
  else
  {
    // Create a brand new Id
    id = this->dataPtr->entities.size();
    this->dataPtr->entities.push_back(std::move(Entity(id)));
  }

  return id;
}

/////////////////////////////////////////////////
bool ComponentManager::EraseEntity(EntityId _id)
{
  bool success = false;
  if (this->HasEntity(_id))
  {
    // \todo(nkoenig) Remove the entity at a good point in the update cycle
    // (aka "toDeleteEntities"), and delete the components associated with
    // the entity.
    success = true;
  }
  return success;
}

/////////////////////////////////////////////////
void ComponentManager::EraseEntities()
{
  this->dataPtr->entities.clear();
  this->dataPtr->entityComponents.clear();
  this->dataPtr->availableEntityIds.clear();

  for (std::pair<const ComponentTypeId,
       std::unique_ptr<ComponentStorageBase>> &comp: this->components)
  {
    comp.second->RemoveAll();
  }
}

/////////////////////////////////////////////////
bool ComponentManager::RemoveComponent(
    const EntityId _id, const ComponentKey &_key)
{
  // Make sure the entity exists and has the component.
  if (this->EntityHasComponent(_id, _key))
  {
    auto entityComponentIter = std::find(
        this->dataPtr->entityComponents[_id].begin(),
        this->dataPtr->entityComponents[_id].end(), _key);

    this->components.at(_key.first)->Remove(_key.second);
    this->dataPtr->entityComponents[_id].erase(entityComponentIter);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool ComponentManager::EntityHasComponent(EntityId _id,
    const ComponentKey &_key) const
{
  return this->HasEntity(_id) &&
    std::find(this->dataPtr->entityComponents[_id].begin(),
        this->dataPtr->entityComponents[_id].end(), _key) !=
    this->dataPtr->entityComponents[_id].end();
}

/////////////////////////////////////////////////
bool ComponentManager::EntityHasComponentType(const EntityId _id,
    const ComponentTypeId &_typeId) const
{
  if (!this->HasEntity(_id))
    return false;

  return std::find_if(this->dataPtr->entityComponents.at(_id).begin(),
      this->dataPtr->entityComponents.at(_id).end(),
      [&] (const ComponentKey &_key) {return _key.first == _typeId;}) !=
    this->dataPtr->entityComponents.at(_id).end();
}

/////////////////////////////////////////////////
bool ComponentManager::HasEntity(EntityId _id) const
{
  //\todo(nkoenig) This function needs to be fixed/implemented.
  // True if the vector is big enough to have used this id
  bool isWithinRange = _id >= 0 &&
    _id < static_cast<EntityId>(this->dataPtr->entities.size());
  /*bool isNotDeleted = this->freeEntityIds.find(_id) ==
                      this->freeEntityIds.end() &&
                      this->deletedIds.find(_id) == this->deletedIds.end();
                      */
  return isWithinRange;// && isNotDeleted;
}

/////////////////////////////////////////////////
ComponentKey ComponentManager::CreateComponentImplementation(
    const EntityId _entityId, const ComponentTypeId _componentTypeId,
    const std::any &_data)
{
  // Make sure the component has been registered.
  if (this->components.find(_componentTypeId) == this->components.end())
  {
    igndbg << "Unable to create unregistered component.\n";
    return {kComponentTypeIdInvalid, kComponentIdInvalid};
  }

  // Instantiate the new component.
  ComponentId componentId = this->components[_componentTypeId]->Create(_data);

  ComponentKey componentKey{_componentTypeId, componentId};

  this->dataPtr->entityComponents[_entityId].push_back(componentKey);

  return componentKey;
}

//////////////////////////////////////////////////
EntityQueryId ComponentManager::AddQuery(const EntityQuery &_query)
{
  std::cout << "Adding query\n";

  for (size_t i = 0; i < this->dataPtr->queries.size(); ++i)
  {
    if (_query == this->dataPtr->queries.at(i))
    {
      // Already have this query.
      return i;
    }
  }

  // \todo(nkoenig) Do we really want to copy the query?
  this->dataPtr->queries.push_back(_query);
  EntityQuery &query = this->dataPtr->queries.back();

  EntityQueryId result = this->dataPtr->queries.size() - 1;

  const std::set<ComponentTypeId> &types = _query.ComponentTypes();

  // \todo(nkoenig) Check that the entities vector is always compact,
  // otherwise this loop could check removed entities.
  for (size_t id = 0; id < this->dataPtr->entities.size(); ++id)
  {
    // Check that the entity has the required components
    if (this->dataPtr->EntityMatches(id, types))
    {
      std::cout << "Added Entity[" << id << "] To Query\n";
      query.AddEntity(id);
    }
  }

  return result;
}

/////////////////////////////////////////////////
bool ComponentManagerPrivate::EntityMatches(EntityId _id,
    const std::set<ComponentTypeId> &_types) const
{
  const std::vector<ComponentKey> &components = this->entityComponents.at(_id);

  // \todo(nkoenig) The performance of this coude be improved. Ideally we
  // wouldn't need two loops to confirm that an entity matches a set of
  // types. Itmight be possible to create bitmask for component sets.
  // Fixing this might not be hight priority, unless we expect frequent
  // creation of entities and/or queries.
  for (const ComponentTypeId &type : _types)
  {
    bool found = false;
    for (const ComponentKey &comp : components)
    {
      if (comp.first == type)
      {
        found = true;
        break;
      }
    }
    if (!found)
      return false;
  }

  return true;
}

/////////////////////////////////////////////////
const std::optional<std::reference_wrapper<EntityQuery>>
ComponentManager::Query(const EntityQueryId _index) const
{
  if (_index >= 0 && _index < this->dataPtr->queries.size())
  {
    return std::optional<std::reference_wrapper<EntityQuery>>(
        this->dataPtr->queries[_index]);
  }
  return std::nullopt;
}

/////////////////////////////////////////////////
ComponentTypeId ComponentManager::Type(const std::string &_name) const
{
  std::map<std::string, ComponentTypeId>::const_iterator iter =
    this->componentNameMap.find(_name);

  if (iter != this->componentNameMap.end())
    return iter->second;
  return kComponentTypeIdInvalid;
}
