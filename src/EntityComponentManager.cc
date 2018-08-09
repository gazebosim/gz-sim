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

#include <map>
#include <set>
#include <vector>

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::EntityComponentManagerPrivate
{
  /// \brief Get whether and entity has the given component types.
  /// \param[in] _id Id of the Entity to check.
  /// \param[in] _types Component types to check that the Entity has.
  /// \return True if the given entity has the given types.
  public: bool EntityMatches(EntityId _id,
    const std::set<ComponentTypeId> &_types) const;

  /// \brief Map of component storage classes. The key is a component
  /// type id, and the value is a pointer to the component storage.
  public: std::map<ComponentTypeId,
          std::unique_ptr<ComponentStorageBase>> components;

  /// \brief instances of entities
  public: std::vector<Entity> entities;

  /// \brief deleted entity ids that can be reused
  // \todo(nkoenig) Add this back in when implementing EraseEntity.
  // public: std::set<EntityId> availableEntityIds;

  /// \brief The set of components that each entity has
  public: std::map<EntityId, std::vector<ComponentKey>> entityComponents;

  /// \brief Set of known Entity queries.
  public: std::vector<EntityQuery> queries;
};


//////////////////////////////////////////////////
EntityComponentManager::EntityComponentManager()
  : dataPtr(new EntityComponentManagerPrivate)
{
}

//////////////////////////////////////////////////
EntityComponentManager::~EntityComponentManager()
{
}

//////////////////////////////////////////////////
size_t EntityComponentManager::EntityCount() const
{
  return this->dataPtr->entities.size();
}

/////////////////////////////////////////////////
EntityId EntityComponentManager::CreateEntity()
{
  EntityId id = kNullEntity;

  // \todo(nkoenig) Add this back in when implementing EraseEntity.
  // if (!this->dataPtr->availableEntityIds.empty())
  // {
  //   // Reuse the smallest available EntityId
  //   id = *(this->dataPtr->availableEntityIds.begin());
  //   this->dataPtr->availableEntityIds.erase(
  //       this->dataPtr->availableEntityIds.begin());
  //   this->dataPtr->entities[id] = std::move(Entity(id));
  // }
  // else
  // {
    // Create a brand new Id
    id = this->dataPtr->entities.size();
    this->dataPtr->entities.push_back(std::move(Entity(id)));
  // }

  return id;
}

/////////////////////////////////////////////////
// bool EntityComponentManager::EraseEntity(EntityId _id)
// {
//   bool success = false;
//   if (this->HasEntity(_id))
//   {
//     // \todo(nkoenig) Remove the entity at a good point in the update cycle
//     // (aka "toDeleteEntities"), and delete the components associated with
//     // the entity.
//     success = true;
//   }
//   return success;
// }

/////////////////////////////////////////////////
void EntityComponentManager::EraseEntities()
{
  this->dataPtr->entities.clear();
  this->dataPtr->entityComponents.clear();
  // \todo(nkoenig) Add this back in when implementing EraseEntity.
  // this->dataPtr->availableEntityIds.clear();

  for (std::pair<const ComponentTypeId,
       std::unique_ptr<ComponentStorageBase>> &comp: this->dataPtr->components)
  {
    comp.second->RemoveAll();
  }
}

/////////////////////////////////////////////////
bool EntityComponentManager::RemoveComponent(
    const EntityId _id, const ComponentKey &_key)
{
  // Make sure the entity exists and has the component.
  if (this->EntityHasComponent(_id, _key))
  {
    auto entityComponentIter = std::find(
        this->dataPtr->entityComponents[_id].begin(),
        this->dataPtr->entityComponents[_id].end(), _key);

    this->dataPtr->components.at(_key.first)->Remove(_key.second);
    this->dataPtr->entityComponents[_id].erase(entityComponentIter);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponent(EntityId _id,
    const ComponentKey &_key) const
{
  return this->HasEntity(_id) &&
    std::find(this->dataPtr->entityComponents[_id].begin(),
        this->dataPtr->entityComponents[_id].end(), _key) !=
    this->dataPtr->entityComponents[_id].end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponentType(const EntityId _id,
    const ComponentTypeId &_typeId) const
{
  if (!this->HasEntity(_id))
    return false;

  std::map<EntityId, std::vector<ComponentKey>>::const_iterator iter =
    this->dataPtr->entityComponents.find(_id);

  if (iter == this->dataPtr->entityComponents.end())
    return false;

  return std::find_if(iter->second.begin(), iter->second.end(),
      [&] (const ComponentKey &_key) {return _key.first == _typeId;}) !=
    iter->second.end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasEntity(EntityId _id) const
{
  // \todo(nkoenig) This function needs to be fixed/implemented.
  // True if the vector is big enough to have used this id
  bool isWithinRange = _id >= 0 &&
    _id < static_cast<EntityId>(this->dataPtr->entities.size());
  /* bool isNotDeleted = this->freeEntityIds.find(_id) ==
                      this->freeEntityIds.end() &&
                      this->deletedIds.find(_id) == this->deletedIds.end();
  */
  return isWithinRange;  // && isNotDeleted;
}

/////////////////////////////////////////////////
ComponentKey EntityComponentManager::CreateComponentImplementation(
    const EntityId _entityId, const ComponentTypeId _componentTypeId,
    const std::any &_data)
{
  // Instantiate the new component.
  ComponentId componentId =
    this->dataPtr->components[_componentTypeId]->Create(_data);

  ComponentKey componentKey{_componentTypeId, componentId};

  this->dataPtr->entityComponents[_entityId].push_back(componentKey);

  return componentKey;
}

//////////////////////////////////////////////////
EntityQueryId EntityComponentManager::AddQuery(const EntityQuery &_query)
{
  for (size_t i = 0; i < this->dataPtr->queries.size(); ++i)
  {
    if (_query == this->dataPtr->queries.at(i))
    {
      // Already have this query.
      return i;
    }
  }

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
      query.AddEntity(id);
    }
  }

  return result;
}

/////////////////////////////////////////////////
bool EntityComponentManagerPrivate::EntityMatches(EntityId _id,
    const std::set<ComponentTypeId> &_types) const
{
  const std::vector<ComponentKey> &comps = this->entityComponents.at(_id);

  // \todo(nkoenig) The performance of this coude be improved. Ideally we
  // wouldn't need two loops to confirm that an entity matches a set of
  // types. Itmight be possible to create bitmask for component sets.
  // Fixing this might not be hight priority, unless we expect frequent
  // creation of entities and/or queries.
  for (const ComponentTypeId &type : _types)
  {
    bool found = false;
    for (const ComponentKey &comp : comps)
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
EntityComponentManager::Query(const EntityQueryId _index) const
{
  if (_index >= 0 &&
      _index < static_cast<EntityQueryId>(this->dataPtr->queries.size()))
  {
    return std::optional<std::reference_wrapper<EntityQuery>>(
        this->dataPtr->queries[_index]);
  }
  return std::nullopt;
}

/////////////////////////////////////////////////
const void *EntityComponentManager::ComponentImplementation(
    const EntityId _id, const ComponentTypeId _type) const
{
  std::map<EntityId, std::vector<ComponentKey>>::const_iterator ecIter =
    this->dataPtr->entityComponents.find(_id);

  if (ecIter == this->dataPtr->entityComponents.end())
    return nullptr;

  std::vector<ComponentKey>::const_iterator iter =
    std::find_if(ecIter->second.begin(), ecIter->second.end(),
        [&] (const ComponentKey &_key) {return _key.first == _type;});

  if (iter != ecIter->second.end())
    return this->dataPtr->components.at(iter->first)->Component(iter->second);

  return nullptr;
}

/////////////////////////////////////////////////
const void *EntityComponentManager::ComponentImplementation(
    const ComponentKey &_key) const
{
  if (this->dataPtr->components.find(_key.first) !=
      this->dataPtr->components.end())
  {
    return this->dataPtr->components.at(_key.first)->Component(_key.second);
  }
  return nullptr;
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasComponentType(const ComponentTypeId _typeId)
{
  return this->dataPtr->components.find(_typeId) !=
    this->dataPtr->components.end();
}

/////////////////////////////////////////////////
void EntityComponentManager::RegisterComponentType(
    const ComponentTypeId _typeId,
    ComponentStorageBase *_type)
{
  igndbg << "Register new component type " << _typeId << ".\n";
  this->dataPtr->components[_typeId].reset(_type);
}
