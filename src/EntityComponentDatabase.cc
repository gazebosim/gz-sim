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

#include <algorithm>
#include <assert.h>
#include <set>
#include <utility>

#include "ignition/gazebo/EntityComponentDatabase.hh"
#include "ignition/gazebo/EntityQuery.hh"


using namespace ignition::gazebo;

typedef std::pair<EntityId, ComponentType> StorageKey;

class ignition::gazebo::EntityComponentDatabasePrivate
{
  /// \brief entities that are to be created next update
  // public: std::vector<EntityId> toCreateEntities;

  /// \brief entities that are to be deleted next update
  // public: std::set<EntityId> toDeleteEntities;

  /// \brief components that are to be created next update
  // public: std::map<StorageKey, char*> toAddComponents;

  /// \brief components that are to be modified next update
  // public: std::map<StorageKey, char*> toModifyComponents;

  /// \brief components that are to be deleted next update
  // public: std::vector<StorageKey> toRemoveComponents;

  /// \brief components that were deleted before this update
  // public: std::vector<StorageKey> removedComponents;

  /// \brief instances of entities
  public: std::vector<Entity> entities;

  /// \brief deleted entity ids that can be reused
  public: std::set<EntityId> freeEntityIds;

  /// \brief deleted entity ids that can't yet be reused
  // public: std::set<EntityId> deletedIds;

  // TODO better storage of components
  // Map EntityId/ComponentType pair to an index in this->components
  // public: std::map<StorageKey, int> componentIndices;
  // public: std::vector<char*> components;
  // Map EntityId/ComponentType pair to the state of a component
  // public: std::map<StorageKey, Difference> differences;

  /// \brief update queries because this entity's components have changed
  public: void UpdateQueries(EntityId _id);

  /// \brief return true iff the entity exists
  public: bool EntityExists(EntityId _id) const;

  /// \brief check if an entity has these components
  /// \returns true iff entity has all components in the set
  public: bool EntityMatches(EntityId _id,
              const std::set<ComponentTypeId> &_types) const;

  /// \brief Queries on this manager
  public: std::vector<EntityQuery> queries;
};

/////////////////////////////////////////////////
EntityComponentDatabase::EntityComponentDatabase()
: dataPtr(new EntityComponentDatabasePrivate)
{
}

/////////////////////////////////////////////////
EntityComponentDatabase::~EntityComponentDatabase()
{
}

/////////////////////////////////////////////////
std::pair<EntityQueryId, bool> EntityComponentDatabase::AddQuery(
    const EntityQuery &_query)
{
  for (size_t i = 0; i < this->dataPtr->queries.size(); ++i)
  {
    if (this->dataPtr->queries[i] == _query)
    {
      // Already have this query, bail
      return {i, false};
    }
  }

  const std::set<ComponentTypeId> &types = _query.ComponentTypes();
  this->dataPtr->queries.push_back(_query);
  EntityQueryId result = this->dataPtr->queries.size() - 1;
  EntityQuery &query = this->dataPtr->queries.back();

  for (size_t id = 0; id < this->dataPtr->entities.size(); ++id)
  {
    // Check that the entity has the required components
    if (this->dataPtr->EntityMatches(id, types))
    {
      query.AddEntity(id);
    }
  }

  return {result, true};
}

/////////////////////////////////////////////////
bool EntityComponentDatabase::RemoveQuery(const EntityQueryId /*_id*/)
{
  // \todo(nkoenig) This changes the queries vector which invalidates
  // existing EntityQueryIds.

  /*if (_id >= 0 && _id < this->dataPtr->queries.size())
  {
    this->dataPtr->queries.erase(this->dataPtr->queries.begin() + _id);
    return true;
  }
  */

  return false;
}

/////////////////////////////////////////////////
EntityId EntityComponentDatabase::CreateEntity()
{
  EntityId id;
  if (!this->dataPtr->freeEntityIds.empty())
  {
    // Reuse the smallest deleted EntityId
    id = *(this->dataPtr->freeEntityIds.begin());
    this->dataPtr->freeEntityIds.erase(this->dataPtr->freeEntityIds.begin());
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
bool EntityComponentDatabase::DeleteEntity(EntityId _id)
{
  bool success = false;
  if (this->dataPtr->EntityExists(_id))
  {
    // \todo(nkoenig) Remove the entity at a good point in the update cycle
    // (aka "toDeleteEntities"), and delete the components associated with
    // the entity.
    success = true;
  }
  return success;
}


/////////////////////////////////////////////////
/*ignition::gazebo::Entity &EntityComponentDatabase::Entity(EntityId _id) const
{
  if (this->dataPtr->EntityExists(_id))
    return this->dataPtr->entities[_id];
  else
  {
    return kNullEntity;
  }
}*/

/////////////////////////////////////////////////
/*void *EntityComponentDatabase::AddComponent(EntityId _id, ComponentType _type)
{
  void *component = nullptr;
  StorageKey key = std::make_pair(_id, _type);
  // if component has not been added already
  if (this->dataPtr->componentIndices.find(key) ==
      this->dataPtr->componentIndices.end() &&
      this->dataPtr->toAddComponents.find(key) ==
      this->dataPtr->toAddComponents.end())
  {
    // Allocate memory and call constructor
    ComponentTypeInfo info = ComponentFactory::TypeInfo(_type);
    // TODO store components of same type in adjacent memory
    char *storage = new char[info.size];
    component = static_cast<void *>(storage);
    info.constructor(component);

    this->dataPtr->toAddComponents[key] = storage;
  }

  return component;
}

/////////////////////////////////////////////////
bool EntityComponentDatabase::RemoveComponent(EntityId _id, ComponentType _type)
{
  bool success = false;
  StorageKey key = std::make_pair(_id, _type);
  auto kvIter = this->dataPtr->componentIndices.find(key);
  if (kvIter != this->dataPtr->componentIndices.end())
  {
    // Check if it has already been removed
    if (!std::binary_search(this->dataPtr->toRemoveComponents.begin(),
          this->dataPtr->toRemoveComponents.end(), key))
    {
      // Flag this for removal
      this->dataPtr->toRemoveComponents.push_back(key);
      std::sort(this->dataPtr->toRemoveComponents.begin(),
          this->dataPtr->toRemoveComponents.end());
    }
    success = true;
  }

  return success;
}

/////////////////////////////////////////////////
void const *EntityComponentDatabase::EntityComponent(EntityId _id,
    ComponentType _type) const
{
  void const *component = nullptr;
  StorageKey key = std::make_pair(_id, _type);
  if (this->dataPtr->componentIndices.find(key) !=
      this->dataPtr->componentIndices.end())
  {
    auto index = this->dataPtr->componentIndices[key];
    char *data = this->dataPtr->components[index];
    component = static_cast<void const *>(data);
  }
  return component;
}

/////////////////////////////////////////////////
void *EntityComponentDatabase::EntityComponentMutable(EntityId _id,
    ComponentType _type)
{
  void *component = nullptr;
  StorageKey key = std::make_pair(_id, _type);
  auto compIter = this->dataPtr->componentIndices.find(key);
  if (compIter != this->dataPtr->componentIndices.end())
  {
    auto modIter = this->dataPtr->toModifyComponents.find(key);
    if (modIter != this->dataPtr->toModifyComponents.end())
    {
      // Already been modified, return pointer to new storage
      char *storage = modIter->second;
      component = static_cast<void *>(storage);
    }
    else
    {
      char const *readOnlyStorage = this->dataPtr->components[compIter->second];
      void const *readOnlyComp = static_cast<void const *>(readOnlyStorage);
      // create temporary storage for the updated data
      ComponentTypeInfo info = ComponentFactory::TypeInfo(_type);
      char *storage = new char[info.size];
      component = static_cast<void *>(storage);
      info.deepCopier(readOnlyComp, component);
      this->dataPtr->toModifyComponents[key] = storage;
    }
  }
  return component;
}



//////////////////////////////////////////////////
void EntityComponentDatabase::InstantQuery(EntityQuery &_query)
{
  for (const ignition::gazebo::Entity &entity : this->dataPtr->entities)
  {
    if (this->dataPtr->EntityMatches(entity.Id(), _query.ComponentTypes()))
      _query.AddEntity(entity.Id());
  }
}
*/

/////////////////////////////////////////////////
/*const EntityQuery &EntityComponentDatabase::Query(
    const EntityQueryId _index) const
{
  if (_index >= 0 && _index < this->dataPtr->queries.size())
    return this->dataPtr->queries[_index];
  return kEntityQueryNull;
}*/

/////////////////////////////////////////////////
/*Difference EntityComponentDatabase::IsDifferent(EntityId _id,
    ComponentType _type) const
{
  Difference d = NO_DIFFERENCE;
  StorageKey key = std::make_pair(_id, _type);
  auto iter = this->dataPtr->differences.find(key);
  if (iter != this->dataPtr->differences.end())
  {
    d = iter->second;
  }
  return d;
}*/

/////////////////////////////////////////////////
void EntityComponentDatabase::Update()
{
  // Deleted ids can be reused after one update.
  /*this->dataPtr->freeEntityIds.insert(this->dataPtr->deleteEntitydIds.begin(),
      this->dataPtr->deletedEntityIds.end());

  // Move toDeleteEntities to deletedIds, effectively deleting them
  this->dataPtr->deletedIds = std::move(this->dataPtr->toDeleteEntities);

  this->dataPtr->differences.clear();
  */

  // Modify components
  /*for (auto const &kv : this->dataPtr->toModifyComponents)
  {
    StorageKey key = kv.first;
    this->dataPtr->differences[key] = WAS_MODIFIED;
    ComponentTypeInfo info = ComponentFactory::TypeInfo(key.second);

    // Get pointer to temporary storage with modifications
    char *modifiedStorage = kv.second;

    // Get pointer to component in main storage
    auto mainIdx = this->dataPtr->componentIndices[key];
    char *mainStorage = this->dataPtr->components[mainIdx];

    // destruct old component in main storage
    info.destructor(static_cast<void *>(modifiedStorage));

    // Copy modified component to main storage
    info.shallowCopier(static_cast<const void *>(modifiedStorage),
        static_cast<void *>(mainStorage));

    // Free space used for modified component
    delete [] modifiedStorage;
  }
  this->dataPtr->toModifyComponents.clear();

  // Remove the components for real
  for (StorageKey key : this->dataPtr->toRemoveComponents)
  {
    int index = this->dataPtr->componentIndices.find(key)->second;
    // call destructor
    void *component = nullptr;
    ComponentTypeInfo info = ComponentFactory::TypeInfo(key.second);
    char *storage = this->dataPtr->components[index];
    component = static_cast<void *>(storage);
    info.destructor(component);

    this->dataPtr->differences[key] = WAS_DELETED;

    delete [] storage;
    this->dataPtr->components.erase(this->dataPtr->components.begin() + index);
    this->dataPtr->componentIndices.erase(key);
    // Update the indexes beyond
    for (auto &kvIter : this->dataPtr->componentIndices)
    {
      int &otherIndex = kvIter.second;
      if (otherIndex > index)
        --otherIndex;
    }
  }

  // Update queries with components removed more than 1 update ago
  for (StorageKey key : this->dataPtr->removedComponents)
  {
    for (auto &query : this->dataPtr->queries)
    {
      auto const & types = query.ComponentTypes();
      if (types.find(key.second) != types.end())
        query.RemoveEntity(key.first);
    }
  }
  this->dataPtr->removedComponents = std::move(
      this->dataPtr->toRemoveComponents);

  // Update querys with added components
  for (auto kv : this->dataPtr->toAddComponents)
  {
    char *storage = kv.second;
    StorageKey key = kv.first;
    EntityId id = key.first;
    this->dataPtr->differences[key] = WAS_CREATED;
    // Add to main storage
    auto index = this->dataPtr->components.size();
    this->dataPtr->components.push_back(storage);
    this->dataPtr->componentIndices[key] = index;
    this->dataPtr->UpdateQueries(id);
  }
  this->dataPtr->toAddComponents.clear();

  // Clearing this effectively creates entities
  this->dataPtr->toCreateEntities.clear();

  assert(this->dataPtr->componentIndices.size()
      == this->dataPtr->components.size());
  */
}

/////////////////////////////////////////////////
bool EntityComponentDatabasePrivate::EntityExists(EntityId _id) const
{
  // True if the vector is big enough to have used this id
  bool isWithinRange = _id >= 0 &&
    _id < static_cast<EntityId>(this->entities.size());
  /*bool isNotDeleted = this->freeEntityIds.find(_id) ==
                      this->freeEntityIds.end() &&
                      this->deletedIds.find(_id) == this->deletedIds.end();
                      */
  return isWithinRange;// && isNotDeleted;
}

/////////////////////////////////////////////////
void EntityComponentDatabasePrivate::UpdateQueries(EntityId _id)
{
  // \todo(nkoenig) remove entity from query if the entity's components no
  // longer match.
  for (auto &query : this->queries)
  {
    if (this->EntityMatches(_id, query.ComponentTypes()))
      query.AddEntity(_id);
  }
}

/////////////////////////////////////////////////
bool EntityComponentDatabasePrivate::EntityMatches(EntityId /*_id*/,
    const std::set<ComponentTypeId> &/*_types*/) const
{
  bool found = true;
  /*for (const ComponentTypeId &type : _types)
  {
    StorageKey key = std::make_pair(_id, type);
    if (this->componentIndices.find(key) == this->componentIndices.end() &&
        std::find(this->removedComponents.begin(),
          this->removedComponents.end(), key) == this->removedComponents.end())
    {
      found = false;
      break;
    }
  }*/
  return found;
}
