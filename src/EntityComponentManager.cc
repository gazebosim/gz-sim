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

#include "ignition/common/Profiler.hh"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::EntityComponentManagerPrivate
{
  /// \brief Map of component storage classes. The key is a component
  /// type id, and the value is a pointer to the component storage.
  public: std::map<ComponentTypeId,
          std::unique_ptr<ComponentStorageBase>> components;

  /// \brief Instances of entities
  public: std::vector<Entity> entities;

  /// \brief Entities that have just been created
  public: std::set<Entity> newlyCreatedEntities;

  /// \brief Deleted entities that can be reused
  public: std::set<Entity> availableEntities;

  /// \brief Entities that need to be erased.
  public: std::set<Entity> toEraseEntities;

  /// \brief Flag that indicates if all entities should be erased.
  public: bool eraseAllEntities{false};

  /// \brief The set of components that each entity has
  public: std::map<Entity, std::vector<ComponentKey>> entityComponents;

  /// \brief A mutex to protect newly created entityes.
  public: std::mutex entityCreatedMutex;

  /// \brief A mutex to protect entity erase.
  public: std::mutex entityEraseMutex;

  /// \brief The set of all views.
  public: mutable std::map<ComponentTypeKey, View> views;
};

//////////////////////////////////////////////////
EntityComponentManager::EntityComponentManager()
  : dataPtr(new EntityComponentManagerPrivate)
{
}

//////////////////////////////////////////////////
EntityComponentManager::~EntityComponentManager() = default;

//////////////////////////////////////////////////
size_t EntityComponentManager::EntityCount() const
{
  return this->dataPtr->entities.size() -
    this->dataPtr->availableEntities.size();
}

/////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntity()
{
  Entity entity = kNullEntity;

  if (!this->dataPtr->availableEntities.empty())
  {
    // Reuse the smallest available Entity
    entity = *(this->dataPtr->availableEntities.begin());
    this->dataPtr->availableEntities.erase(
        this->dataPtr->availableEntities.begin());
    this->dataPtr->entities[entity] = entity;
  }
  else
  {
    // Create a brand new entity
    entity = this->dataPtr->entities.size();
    this->dataPtr->entities.push_back(entity);
  }

  // Add entity to the list of newly created entities
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
    this->dataPtr->newlyCreatedEntities.insert(entity);
  }

  return entity;
}

/////////////////////////////////////////////////
void EntityComponentManager::ClearNewlyCreatedEntities()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
  this->dataPtr->newlyCreatedEntities.clear();
  for (std::pair<const ComponentTypeKey, View> &view : this->dataPtr->views)
  {
    view.second.ClearNewEntities();
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestEraseEntity(Entity _entity)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->entityEraseMutex);
    this->dataPtr->toEraseEntities.insert(_entity);
  }
  this->UpdateViews(_entity);
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestEraseEntities()
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->entityEraseMutex);
    this->dataPtr->eraseAllEntities = true;
  }
  this->RebuildViews();
}

/////////////////////////////////////////////////
void EntityComponentManager::ProcessEraseEntityRequests()
{
  IGN_PROFILE("EntityComponentManager::ProcessEraseEntityRequests");
  std::lock_guard<std::mutex> lock(this->dataPtr->entityEraseMutex);
  // Short-cut if erasing all entities
  if (this->dataPtr->eraseAllEntities)
  {
    IGN_PROFILE("EraseAll");
    this->dataPtr->eraseAllEntities = false;
    this->dataPtr->entities.clear();
    this->dataPtr->entityComponents.clear();
    this->dataPtr->availableEntities.clear();
    this->dataPtr->toEraseEntities.clear();

    for (std::pair<const ComponentTypeId,
        std::unique_ptr<ComponentStorageBase>> &comp: this->dataPtr->components)
    {
      comp.second->RemoveAll();
    }

    // All views are now invalid.
    this->dataPtr->views.clear();
  }
  else
  {
    IGN_PROFILE("Erase");
    // Otherwise iterate through the list of entities to erase.
    for (const Entity entity : this->dataPtr->toEraseEntities)
    {
      // Make sure the entity exists and is not erased.
      if (!this->HasEntity(entity))
        continue;

      // Insert the entity into the set of available entities.
      this->dataPtr->availableEntities.insert(entity);

      // Remove the components, if any.
      if (this->dataPtr->entityComponents.find(entity) !=
          this->dataPtr->entityComponents.end())
      {
        for (const ComponentKey &key :
            this->dataPtr->entityComponents.at(entity))
        {
          this->dataPtr->components.at(key.first)->Remove(key.second);
        }

        // Remove the entry in the entityComponent map
        this->dataPtr->entityComponents.erase(entity);
      }

      // Remove the entity from views.
      for (std::pair<const ComponentTypeKey, View> &view : this->dataPtr->views)
      {
        view.second.EraseEntity(entity, view.first);
      }
    }
    // Clear the set of entities to erase.
    this->dataPtr->toEraseEntities.clear();
  }
}

/////////////////////////////////////////////////
bool EntityComponentManager::RemoveComponent(
    const Entity _entity, const ComponentKey &_key)
{
  // Make sure the entity exists and has the component.
  if (!this->EntityHasComponent(_entity, _key))
    return false;

  auto entityComponentIter = std::find(
      this->dataPtr->entityComponents[_entity].begin(),
      this->dataPtr->entityComponents[_entity].end(), _key);

  this->dataPtr->components.at(_key.first)->Remove(_key.second);
  this->dataPtr->entityComponents[_entity].erase(entityComponentIter);

  this->UpdateViews(_entity);
  return true;
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponent(const Entity _entity,
    const ComponentKey &_key) const
{
  return this->HasEntity(_entity) &&
    std::find(this->dataPtr->entityComponents[_entity].begin(),
        this->dataPtr->entityComponents[_entity].end(), _key) !=
    this->dataPtr->entityComponents[_entity].end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponentType(const Entity _entity,
    const ComponentTypeId &_typeId) const
{
  if (!this->HasEntity(_entity))
    return false;

  auto iter = this->dataPtr->entityComponents.find(_entity);

  if (iter == this->dataPtr->entityComponents.end())
    return false;

  return std::find_if(iter->second.begin(), iter->second.end(),
      [&] (const ComponentKey &_key)
      {
        return _key.first == _typeId;
      }) != iter->second.end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::IsNewEntity(const Entity _entity) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
  return this->dataPtr->newlyCreatedEntities.find(_entity) !=
         this->dataPtr->newlyCreatedEntities.end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::IsMarkedForErasure(const Entity _entity) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityEraseMutex);
  if (this->dataPtr->eraseAllEntities)
  {
    return true;
  }
  return this->dataPtr->toEraseEntities.find(_entity) !=
         this->dataPtr->toEraseEntities.end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasEntity(const Entity _entity) const
{
  return
    // Check that the _entity is in range
    _entity >= 0 &&
    _entity < static_cast<Entity>(this->dataPtr->entities.size())
    // Check that the _entity is not deleted (not in the available entity set)
    && this->dataPtr->availableEntities.find(_entity) ==
       this->dataPtr->availableEntities.end();
}

/////////////////////////////////////////////////
ComponentKey EntityComponentManager::CreateComponentImplementation(
    const Entity _entity, const ComponentTypeId _componentTypeId,
    const void *_data)
{
  // Instantiate the new component.
  std::pair<ComponentId, bool> componentIdPair =
    this->dataPtr->components[_componentTypeId]->Create(_data);

  ComponentKey componentKey{_componentTypeId, componentIdPair.first};

  this->dataPtr->entityComponents[_entity].push_back(componentKey);

  if (componentIdPair.second)
    this->RebuildViews();
  else
    this->UpdateViews(_entity);

  return componentKey;
}


/////////////////////////////////////////////////
bool EntityComponentManager::EntityMatches(Entity _entity,
    const std::set<ComponentTypeId> &_types) const
{
  auto iter = this->dataPtr->entityComponents.find(_entity);
  if (iter == this->dataPtr->entityComponents.end())
    return false;

  // \todo(nkoenig) The performance of this could be improved. Ideally we
  // wouldn't need two loops to confirm that an entity matches a set of
  // types. It might be possible to create bitmask for component sets.
  // Fixing this might not be high priority, unless we expect frequent
  // creation of entities and/or queries.
  for (const ComponentTypeId &type : _types)
  {
    bool found = false;
    for (const ComponentKey &comp : iter->second)
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
ComponentId EntityComponentManager::EntityComponentIdFromType(
    const Entity _entity, const ComponentTypeId _type) const
{
  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return -1;

  auto iter =
    std::find_if(ecIter->second.begin(), ecIter->second.end(),
      [&] (const ComponentKey &_key) {return _key.first == _type;});

  if (iter != ecIter->second.end())
    return iter->second;

  return -1;
}

/////////////////////////////////////////////////
const void *EntityComponentManager::ComponentImplementation(
    const Entity _entity, const ComponentTypeId _type) const
{
  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return nullptr;

  auto iter =
    std::find_if(ecIter->second.begin(), ecIter->second.end(),
      [&] (const ComponentKey &_key) {return _key.first == _type;});

  if (iter != ecIter->second.end())
    return this->dataPtr->components.at(iter->first)->Component(iter->second);

  return nullptr;
}

/////////////////////////////////////////////////
void *EntityComponentManager::ComponentImplementation(
    const Entity _entity, const ComponentTypeId _type)
{
  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return nullptr;

  auto iter =
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
void *EntityComponentManager::ComponentImplementation(const ComponentKey &_key)
{
  if (this->dataPtr->components.find(_key.first) !=
      this->dataPtr->components.end())
  {
    return this->dataPtr->components.at(_key.first)->Component(_key.second);
  }
  return nullptr;
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasComponentType(
    const ComponentTypeId _typeId) const
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

/////////////////////////////////////////////////
void *EntityComponentManager::First(const ComponentTypeId _componentTypeId)
{
  auto iter = this->dataPtr->components.find(_componentTypeId);
  if (iter != this->dataPtr->components.end())
  {
    return iter->second->First();
  }
  return nullptr;
}

//////////////////////////////////////////////////
std::vector<Entity> &EntityComponentManager::Entities() const
{
  return this->dataPtr->entities;
}

//////////////////////////////////////////////////
bool EntityComponentManager::FindView(const std::set<ComponentTypeId> &_types,
    std::map<ComponentTypeKey, View>::iterator &_iter) const
{
  _iter = this->dataPtr->views.find(_types);
  return _iter != this->dataPtr->views.end();
}

//////////////////////////////////////////////////
std::map<ComponentTypeKey, View>::iterator EntityComponentManager::AddView(
    const std::set<ComponentTypeId> &_types, View &&_view) const
{
  // If the view already exists, then the map will return the iterator to
  // the location that prevented the insertion.
  return this->dataPtr->views.insert(
      std::make_pair(_types, std::move(_view))).first;
}

//////////////////////////////////////////////////
void EntityComponentManager::UpdateViews(const Entity _entity)
{
  IGN_PROFILE("EntityComponentManager::UpdateViews");
  for (std::pair<const ComponentTypeKey, View> &view : this->dataPtr->views)
  {
    // Add/update the entity if it matches the view.
    if (this->EntityMatches(_entity, view.first))
    {
      view.second.AddEntity(_entity, this->IsNewEntity(_entity));
      // If there is a request to delete this entity, update the view as
      // well
      if (this->IsMarkedForErasure(_entity))
      {
        view.second.AddEntityToErased(_entity);
      }
      for (const ComponentTypeId &compTypeId : view.first)
      {
        view.second.AddComponent(_entity, compTypeId,
            this->EntityComponentIdFromType(_entity, compTypeId));
      }
    }
    else
    {
      view.second.EraseEntity(_entity, view.first);
    }
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::RebuildViews()
{
  IGN_PROFILE("EntityComponentManager::RebuildViews");
  for (std::pair<const ComponentTypeKey, View> &view : this->dataPtr->views)
  {
    view.second.entities.clear();
    view.second.components.clear();
    // Add all the entities that match the component types to the
    // view.
    for (const Entity &entity : this->dataPtr->entities)
    {
      if (this->EntityMatches(entity, view.first))
      {
        view.second.AddEntity(entity, this->IsNewEntity(entity));
        // If there is a request to delete this entity, update the view as
        // well
        if (this->IsMarkedForErasure(entity))
        {
          view.second.AddEntityToErased(entity);
        }
        // Store pointers to all the components. This recursively adds
        // all the ComponentTypeTs that belong to the entity to the view.
        for (const ComponentTypeId &compTypeId : view.first)
        {
          view.second.AddComponent(entity, compTypeId,
              this->EntityComponentIdFromType(
                entity, compTypeId));
        }
      }
    }
  }
}

