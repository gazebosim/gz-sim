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

#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/graph/GraphAlgorithms.hh>
#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::EntityComponentManagerPrivate
{
  /// \brief Implementation of the CreateEntity function, which takes a specific
  /// entity as input.
  /// \param[in] _entity Entity to be created.
  /// \return Created entity, which should match the input.
  public: Entity CreateEntityImplementation(Entity _entity);

  /// \brief Recursively insert an entity and all its descendants into a given
  /// set.
  /// \param[in] _entity Entity to be inserted.
  /// \param[in, out] _set Set to be filled.
  public: void InsertEntityRecursive(Entity _entity,
      std::unordered_set<Entity> &_set);

  /// \brief Register a new component type.
  /// \param[in] _typeId Type if of the new component.
  /// \return True if created successfully.
  public: bool CreateComponentStorage(const ComponentTypeId _typeId);

  /// \brief Allots the work for multiple threads prior to running
  /// `AddEntityToMessage`.
  public: void CalculateStateThreadLoad();

  /// \brief Create a message for the removed components
  /// \param[in] _entity Entity with the removed components
  /// \param[in, out] _msg Entity message
  /// \param[in] _types _types Type IDs of components to be serialized. Leave
  /// empty to get all removed components.
  public: void SetRemovedComponentsMsgs(Entity &_entity,
      msgs::SerializedEntity *_msg,
      const std::unordered_set<ComponentTypeId> &_types = {});

  /// \brief Create a message for the removed components
  /// \param[in] _entity Entity with the removed components
  /// \param[in, out] _msg State message
  /// \param[in] _types _types Type IDs of components to be serialized. Leave
  /// empty to get all removed components.
  public: void SetRemovedComponentsMsgs(Entity &_entity,
      msgs::SerializedStateMap &_msg,
      const std::unordered_set<ComponentTypeId> &_types = {});

  /// \brief Add newly modified (created/modified/removed) components to
  /// modifiedComponents list. The entity is added to the list when it is not
  /// a newly created entity or is not an entity to be removed
  /// \param[in] _entity Entity that has component newly modified
  public: void AddModifiedComponent(const Entity &_entity);

  /// \brief Map of component storage classes. The key is a component
  /// type id, and the value is a pointer to the component storage.
  public: std::unordered_map<ComponentTypeId,
          std::unique_ptr<ComponentStorageBase>> components;

  /// \brief A graph holding all entities, arranged according to their
  /// parenting.
  public: EntityGraph entities;

  /// \brief Components that have been changed through a peridic change.
  public: std::set<ComponentKey> periodicChangedComponents;

  /// \brief Components that have been changed through a one-time change.
  public: std::set<ComponentKey> oneTimeChangedComponents;

  /// \brief Entities that have just been created
  public: std::unordered_set<Entity> newlyCreatedEntities;

  /// \brief Entities that need to be removed.
  public: std::unordered_set<Entity> toRemoveEntities;

  /// \brief Entities that have components newly modified
  /// (created/modified/removed) but are not entities that have been
  /// newly created or removed (ie. newlyCreatedEntities or toRemoveEntities).
  /// This is used for the ChangedState functions
  public: std::unordered_set<Entity> modifiedComponents;

  /// \brief Flag that indicates if all entities should be removed.
  public: bool removeAllEntities{false};

  /// \brief True if the entityComponents map was changed.  Primarily used
  /// by the multithreading functionality in `State()` to allocate work to
  /// each thread.
  public: bool entityComponentsDirty{true};

  /// \brief The set of components that each entity has.
  /// NOTE: Any modification of this data structure must be followed
  /// by setting `entityComponentsDirty` to true.
  public: std::unordered_map<Entity,
          std::unordered_map<ComponentTypeId, ComponentId>> entityComponents;

  /// \brief A vector of iterators to evenly distributed spots in the
  /// `entityComponents` map.  Threads in the `State` function use this
  /// vector for easy access of their pre-allocated work.  This vector
  /// is recalculated if `entityComponents` is changed (when
  /// `entityComponentsDirty` == true).
  public: std::vector<std::unordered_map<Entity,
          std::unordered_map<ComponentTypeId, ComponentId>>::iterator>
            entityComponentIterators;

  /// \brief A mutex to protect newly created entities.
  public: std::mutex entityCreatedMutex;

  /// \brief A mutex to protect entity remove.
  public: std::mutex entityRemoveMutex;

  /// \brief A mutex to protect from concurrent writes to views
  public: mutable std::mutex viewsMutex;

  /// \brief A mutex to protect removed components
  public: mutable std::mutex removedComponentsMutex;

  /// \brief The set of all views.
  public: mutable std::map<detail::ComponentTypeKey, detail::View> views;

  /// \brief Cache of previously queried descendants. The key is the parent
  /// entity for which descendants were queried, and the value are all its
  /// descendants.
  public: mutable std::unordered_map<Entity, std::unordered_set<Entity>>
          descendantCache;

  /// \brief Keep track of entities already used to ensure uniqueness.
  public: uint64_t entityCount{0};

  /// \brief Unordered multimap of removed components. The key is the entity to
  /// which belongs the component, and the value is the component being
  /// removed.
  std::unordered_multimap<Entity, ComponentKey> removedComponents;
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
  return this->dataPtr->entities.Vertices().size();
}

/////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntity()
{
  Entity entity = ++this->dataPtr->entityCount;

  if (entity == std::numeric_limits<uint64_t>::max())
  {
    ignwarn << "Reached maximum number of entities [" << entity << "]"
            << std::endl;
    return entity;
  }

  return this->dataPtr->CreateEntityImplementation(entity);
}

/////////////////////////////////////////////////
Entity EntityComponentManagerPrivate::CreateEntityImplementation(Entity _entity)
{
  IGN_PROFILE("EntityComponentManager::CreateEntityImplementation");
  this->entities.AddVertex(std::to_string(_entity), _entity, _entity);

  // Add entity to the list of newly created entities
  {
    std::lock_guard<std::mutex> lock(this->entityCreatedMutex);
    this->newlyCreatedEntities.insert(_entity);
  }

  // Reset descendants cache
  this->descendantCache.clear();

  return _entity;
}

/////////////////////////////////////////////////
void EntityComponentManager::ClearNewlyCreatedEntities()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
  this->dataPtr->newlyCreatedEntities.clear();

  for (auto &view : this->dataPtr->views)
  {
    view.second.ClearNewEntities();
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::ClearRemovedComponents()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->removedComponentsMutex);
  this->dataPtr->removedComponents.clear();
}

/////////////////////////////////////////////////
void EntityComponentManagerPrivate::InsertEntityRecursive(Entity _entity,
    std::unordered_set<Entity> &_set)
{
  for (const auto &vertex : this->entities.AdjacentsFrom(_entity))
  {
    this->InsertEntityRecursive(vertex.first, _set);
  }
  _set.insert(_entity);
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestRemoveEntity(Entity _entity,
    bool _recursive)
{
  // Store the to-be-removed entities in a temporary set so we can call
  // UpdateViews on each of them
  std::unordered_set<Entity> tmpToRemoveEntities;
  if (!_recursive)
  {
    tmpToRemoveEntities.insert(_entity);
  }
  else
  {
    this->dataPtr->InsertEntityRecursive(_entity, tmpToRemoveEntities);
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
    this->dataPtr->toRemoveEntities.insert(tmpToRemoveEntities.begin(),
                                          tmpToRemoveEntities.end());
  }

  for (const auto &removedEntity : tmpToRemoveEntities)
  {
    this->UpdateViews(removedEntity);
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestRemoveEntities()
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
    this->dataPtr->removeAllEntities = true;
  }
  this->RebuildViews();
}

/////////////////////////////////////////////////
void EntityComponentManager::ProcessRemoveEntityRequests()
{
  IGN_PROFILE("EntityComponentManager::ProcessRemoveEntityRequests");
  std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
  // Short-cut if erasing all entities
  if (this->dataPtr->removeAllEntities)
  {
    IGN_PROFILE("RemoveAll");
    this->dataPtr->removeAllEntities = false;
    this->dataPtr->entities = EntityGraph();
    this->dataPtr->entityComponents.clear();
    this->dataPtr->toRemoveEntities.clear();
    this->dataPtr->entityComponentsDirty = true;

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
    IGN_PROFILE("Remove");
    // Otherwise iterate through the list of entities to remove.
    for (const Entity entity : this->dataPtr->toRemoveEntities)
    {
      // Make sure the entity exists and is not removed.
      if (!this->HasEntity(entity))
        continue;

      // Remove from graph
      this->dataPtr->entities.RemoveVertex(entity);

      auto entityIter = this->dataPtr->entityComponents.find(entity);
      // Remove the components, if any.
      if (entityIter != this->dataPtr->entityComponents.end())
      {
        for (const auto &key : entityIter->second)
        {
          this->dataPtr->components.at(key.first)->Remove(key.second);
        }

        // Remove the entry in the entityComponent map
        this->dataPtr->entityComponents.erase(entity);
        this->dataPtr->entityComponentsDirty = true;
      }

      // Remove the entity from views.
      for (auto &view : this->dataPtr->views)
      {
        view.second.RemoveEntity(entity, view.first);
      }
    }
    // Clear the set of entities to remove.
    this->dataPtr->toRemoveEntities.clear();
  }

  // Reset descendants cache
  this->dataPtr->descendantCache.clear();
}

/////////////////////////////////////////////////
bool EntityComponentManager::RemoveComponent(
    const Entity _entity, const ComponentTypeId &_typeId)
{
  auto componentId = this->EntityComponentIdFromType(_entity, _typeId);
  ComponentKey key{_typeId, componentId};
  return this->RemoveComponent(_entity, key);
}

/////////////////////////////////////////////////
bool EntityComponentManager::RemoveComponent(
    const Entity _entity, const ComponentKey &_key)
{
  IGN_PROFILE("EntityComponentManager::RemoveComponent");
  // Make sure the entity exists and has the component.
  if (!this->EntityHasComponent(_entity, _key))
    return false;

  this->dataPtr->components.at(_key.first)->Remove(_key.second);
  this->dataPtr->entityComponents[_entity].erase(_key.first);
  this->dataPtr->oneTimeChangedComponents.erase(_key);
  this->dataPtr->periodicChangedComponents.erase(_key);
  this->dataPtr->entityComponentsDirty = true;

  this->UpdateViews(_entity);

  this->dataPtr->AddModifiedComponent(_entity);

  // Add component to map of removed components
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->removedComponentsMutex);
    this->dataPtr->removedComponents.insert(std::make_pair(_entity, _key));
  }

  return true;
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponent(const Entity _entity,
    const ComponentKey &_key) const
{
  if (!this->HasEntity(_entity))
    return false;
  auto &compMap = this->dataPtr->entityComponents[_entity];
  return compMap.find(_key.first) != compMap.end();
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

  auto typeIter = iter->second.find(_typeId);
  return (typeIter != iter->second.end());
}

/////////////////////////////////////////////////
bool EntityComponentManager::IsNewEntity(const Entity _entity) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
  return this->dataPtr->newlyCreatedEntities.find(_entity) !=
         this->dataPtr->newlyCreatedEntities.end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::IsMarkedForRemoval(const Entity _entity) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
  if (this->dataPtr->removeAllEntities)
  {
    return true;
  }
  return this->dataPtr->toRemoveEntities.find(_entity) !=
         this->dataPtr->toRemoveEntities.end();
}

/////////////////////////////////////////////////
ComponentState EntityComponentManager::ComponentState(const Entity _entity,
    const ComponentTypeId _typeId) const
{
  auto result = ComponentState::NoChange;

  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return result;

  auto typeKey = ecIter->second.find(_typeId);
  if (typeKey == ecIter->second.end())
    return result;

  ComponentKey key{_typeId, typeKey->second};

  if (this->dataPtr->oneTimeChangedComponents.find(key) !=
      this->dataPtr->oneTimeChangedComponents.end())
  {
    result = ComponentState::OneTimeChange;
  }
  else if (this->dataPtr->periodicChangedComponents.find(key) !=
      this->dataPtr->periodicChangedComponents.end())
  {
    result = ComponentState::PeriodicChange;
  }

  return result;
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasNewEntities() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
  return !this->dataPtr->newlyCreatedEntities.empty();
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasEntitiesMarkedForRemoval() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
  return this->dataPtr->removeAllEntities ||
      !this->dataPtr->toRemoveEntities.empty();
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasOneTimeComponentChanges() const
{
  return !this->dataPtr->oneTimeChangedComponents.empty();
}

/////////////////////////////////////////////////
std::unordered_set<ComponentTypeId>
    EntityComponentManager::ComponentTypesWithPeriodicChanges() const
{
  std::unordered_set<ComponentTypeId> periodicComponents;
  for (const auto& compPair : this->dataPtr->periodicChangedComponents)
  {
    periodicComponents.insert(compPair.first);
  }
  return periodicComponents;
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasEntity(const Entity _entity) const
{
  auto vertex = this->dataPtr->entities.VertexFromId(_entity);
  return vertex.Id() != math::graph::kNullId;
}

/////////////////////////////////////////////////
Entity EntityComponentManager::ParentEntity(const Entity _entity) const
{
  auto parents = this->Entities().AdjacentsTo(_entity);
  if (parents.empty())
    return kNullEntity;

  // TODO(louise) Do we want to support multiple parents?
  return parents.begin()->first;
}

/////////////////////////////////////////////////
bool EntityComponentManager::SetParentEntity(const Entity _child,
    const Entity _parent)
{
  // Remove current parent(s)
  auto parents = this->Entities().AdjacentsTo(_child);
  for (const auto &parent : parents)
  {
    auto edge = this->dataPtr->entities.EdgeFromVertices(parent.first, _child);
    this->dataPtr->entities.RemoveEdge(edge);
  }

  // Leave parent-less
  if (_parent == kNullEntity)
  {
    return true;
  }

  // Add edge
  auto edge = this->dataPtr->entities.AddEdge({_parent, _child}, true);
  return (math::graph::kNullId != edge.Id());
}

/////////////////////////////////////////////////
ComponentKey EntityComponentManager::CreateComponentImplementation(
    const Entity _entity, const ComponentTypeId _componentTypeId,
    const components::BaseComponent *_data)
{
  // If type hasn't been instantiated yet, create a storage for it
  if (!this->HasComponentType(_componentTypeId))
  {
    if (!this->dataPtr->CreateComponentStorage(_componentTypeId))
    {
      ignerr << "Failed to create component of type [" << _componentTypeId
             << "] for entity [" << _entity
             << "]. Type has not been properly registered." << std::endl;
      return ComponentKey();
    }
  }

  this->dataPtr->AddModifiedComponent(_entity);

  // Instantiate the new component.
  std::pair<ComponentId, bool> componentIdPair =
    this->dataPtr->components[_componentTypeId]->Create(_data);

  ComponentKey componentKey{_componentTypeId, componentIdPair.first};

  this->dataPtr->entityComponents[_entity].insert(
      {_componentTypeId, componentIdPair.first});
  this->dataPtr->oneTimeChangedComponents.insert(componentKey);
  this->dataPtr->entityComponentsDirty = true;

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

  // \todo(nkoenig) The performance of this could be improved.
  // It might be possible to create bitmask for component sets.
  // Fixing this might not be high priority, unless we expect frequent
  // creation of entities and/or queries.
  for (const ComponentTypeId &type : _types)
  {
    auto typeIter = iter->second.find(type);
    if (typeIter == iter->second.end())
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

  auto typeIter = ecIter->second.find(_type);
  if (typeIter != ecIter->second.end())
    return typeIter->second;

  return -1;
}

/////////////////////////////////////////////////
const components::BaseComponent
    *EntityComponentManager::ComponentImplementation(
    const Entity _entity, const ComponentTypeId _type) const
{
  IGN_PROFILE("EntityComponentManager::ComponentImplementation");
  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return nullptr;

  auto typeIter = ecIter->second.find(_type);
  if (typeIter != ecIter->second.end())
    return this->dataPtr->components.at(_type)->Component(
        typeIter->second);

  return nullptr;
}

/////////////////////////////////////////////////
components::BaseComponent *EntityComponentManager::ComponentImplementation(
    const Entity _entity, const ComponentTypeId _type)
{
  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return nullptr;

  auto typeIter = ecIter->second.find(_type);
  if (typeIter != ecIter->second.end())
    return this->dataPtr->components.at(_type)->Component(typeIter->second);

  return nullptr;
}

/////////////////////////////////////////////////
const components::BaseComponent
    *EntityComponentManager::ComponentImplementation(
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
components::BaseComponent *EntityComponentManager::ComponentImplementation(
    const ComponentKey &_key)
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
bool EntityComponentManagerPrivate::CreateComponentStorage(
    const ComponentTypeId _typeId)
{
  auto storage = components::Factory::Instance()->NewStorage(_typeId);

  if (nullptr == storage)
  {
    ignerr << "Internal errror: failed to create storage for type [" << _typeId
           << "]" << std::endl;
    return false;
  }

  this->components[_typeId] = std::move(storage);
  igndbg << "Using components of type [" << _typeId << "] / ["
         << components::Factory::Instance()->Name(_typeId) << "].\n";

  return true;
}

/////////////////////////////////////////////////
components::BaseComponent *EntityComponentManager::First(
    const ComponentTypeId _componentTypeId)
{
  auto iter = this->dataPtr->components.find(_componentTypeId);
  if (iter != this->dataPtr->components.end())
  {
    return iter->second->First();
  }
  return nullptr;
}

//////////////////////////////////////////////////
const EntityGraph &EntityComponentManager::Entities() const
{
  return this->dataPtr->entities;
}

//////////////////////////////////////////////////
bool EntityComponentManager::FindView(const std::set<ComponentTypeId> &_types,
    std::map<detail::ComponentTypeKey, detail::View>::iterator &_iter) const
{
  std::lock_guard<std::mutex> lockViews(this->dataPtr->viewsMutex);
  _iter = this->dataPtr->views.find(_types);
  return _iter != this->dataPtr->views.end();
}

//////////////////////////////////////////////////
std::map<detail::ComponentTypeKey, detail::View>::iterator
    EntityComponentManager::AddView(const std::set<ComponentTypeId> &_types,
    detail::View &&_view) const
{
  // If the view already exists, then the map will return the iterator to
  // the location that prevented the insertion.
  std::lock_guard<std::mutex> lockViews(this->dataPtr->viewsMutex);
  return this->dataPtr->views.insert(
      std::make_pair(_types, std::move(_view))).first;
}

//////////////////////////////////////////////////
void EntityComponentManager::UpdateViews(const Entity _entity)
{
  IGN_PROFILE("EntityComponentManager::UpdateViews");
  for (auto &view : this->dataPtr->views)
  {
    // Add/update the entity if it matches the view.
    if (this->EntityMatches(_entity, view.first))
    {
      view.second.AddEntity(_entity, this->IsNewEntity(_entity));
      // If there is a request to delete this entity, update the view as
      // well
      if (this->IsMarkedForRemoval(_entity))
      {
        view.second.AddEntityToRemoved(_entity);
      }
      for (const ComponentTypeId &compTypeId : view.first)
      {
        view.second.AddComponent(_entity, compTypeId,
            this->EntityComponentIdFromType(_entity, compTypeId));
      }
    }
    else
    {
      view.second.RemoveEntity(_entity, view.first);
    }
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::RebuildViews()
{
  IGN_PROFILE("EntityComponentManager::RebuildViews");
  for (auto &view : this->dataPtr->views)
  {
    view.second.entities.clear();
    view.second.components.clear();
    // Add all the entities that match the component types to the
    // view.
    for (const auto &vertex : this->dataPtr->entities.Vertices())
    {
      Entity entity = vertex.first;
      if (this->EntityMatches(entity, view.first))
      {
        view.second.AddEntity(entity, this->IsNewEntity(entity));
        // If there is a request to delete this entity, update the view as
        // well
        if (this->IsMarkedForRemoval(entity))
        {
          view.second.AddEntityToRemoved(entity);
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

//////////////////////////////////////////////////
void EntityComponentManagerPrivate::SetRemovedComponentsMsgs(Entity &_entity,
    msgs::SerializedEntity *_entityMsg,
    const std::unordered_set<ComponentTypeId> &_types)
{
  std::lock_guard<std::mutex> lock(this->removedComponentsMutex);
  auto entRemovedComps = this->removedComponents.equal_range(_entity);
  for (auto it = entRemovedComps.first; it != entRemovedComps.second; ++it)
  {
    auto removedComponent = it->second;

    if (!_types.empty() && _types.find(removedComponent.first) == _types.end())
    {
      continue;
    }

    auto compMsg = _entityMsg->add_components();

    // Empty data is needed for the component to be processed afterwards
    compMsg->set_component(" ");
    compMsg->set_type(removedComponent.first);
    compMsg->set_remove(true);
  }
}

//////////////////////////////////////////////////
void EntityComponentManagerPrivate::SetRemovedComponentsMsgs(Entity &_entity,
    msgs::SerializedStateMap &_msg,
    const std::unordered_set<ComponentTypeId> &_types)
{
  std::lock_guard<std::mutex> lock(this->removedComponentsMutex);
  uint64_t nEntityKeys = this->removedComponents.count(_entity);
  if (nEntityKeys == 0)
    return;

  // The message need not necessarily contain the entity initially. For
  // instance, when AddEntityToMessage() calls this function, the entity may
  // have some removed components but none in entityComponents that changed,
  // so the entity may not have been added to the message beforehand.
  auto entIter = _msg.mutable_entities()->find(_entity);
  if (entIter == _msg.mutable_entities()->end())
  {
    msgs::SerializedEntityMap ent;
    ent.set_id(_entity);
    entIter =
      (_msg.mutable_entities())->insert({static_cast<uint64_t>(_entity), ent})
      .first;
  }

  auto entRemovedComps = this->removedComponents.equal_range(_entity);
  for (auto it = entRemovedComps.first; it != entRemovedComps.second; ++it)
  {
    auto removedComponent = it->second;

    if (!_types.empty() && _types.find(removedComponent.first) == _types.end())
    {
      continue;
    }

    msgs::SerializedComponent compMsg;

    // Empty data is needed for the component to be processed afterwards
    compMsg.set_component(" ");
    compMsg.set_type(removedComponent.first);
    compMsg.set_remove(true);

    (*(entIter->second.mutable_components()))[
      static_cast<int64_t>(removedComponent.first)] = compMsg;
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::AddEntityToMessage(msgs::SerializedState &_msg,
    Entity _entity, const std::unordered_set<ComponentTypeId> &_types) const
{
  auto entityMsg = _msg.add_entities();
  entityMsg->set_id(_entity);
  auto iter = this->dataPtr->entityComponents.find(_entity);
  if (iter == this->dataPtr->entityComponents.end())
    return;

  if (this->dataPtr->toRemoveEntities.find(_entity) !=
      this->dataPtr->toRemoveEntities.end())
  {
    entityMsg->set_remove(true);
  }

  // Insert all of the entity's components if the passed in types
  // set is empty
  auto types = _types;
  if (types.empty())
  {
    for (auto &type : this->dataPtr->entityComponents[_entity])
    {
      types.insert(type.first);
    }
  }

  for (const ComponentTypeId type : types)
  {
    // If the entity does not have the component, continue
    auto typeIter = iter->second.find(type);
    if (typeIter == iter->second.end())
    {
      continue;
    }

    auto compMsg = entityMsg->add_components();
    auto compBase = this->ComponentImplementation(_entity, type);
    compMsg->set_type(compBase->TypeId());

    std::ostringstream ostr;
    compBase->Serialize(ostr);

    compMsg->set_component(ostr.str());
  }

  // Add a component to the message and set it to be removed if the component
  // exists in the removedComponents map.
  this->dataPtr->SetRemovedComponentsMsgs(_entity, entityMsg, _types);
}

//////////////////////////////////////////////////
void EntityComponentManager::AddEntityToMessage(msgs::SerializedStateMap &_msg,
    Entity _entity, const std::unordered_set<ComponentTypeId> &_types,
    bool _full) const
{
  auto iter = this->dataPtr->entityComponents.find(_entity);
  if (iter == this->dataPtr->entityComponents.end())
    return;

  // Set the default entity iterator to the end. This will allow us to know
  // if the entity has been added to the message.
  auto entIter = _msg.mutable_entities()->end();
  // Add an entity to the message and set it to be removed if the entity
  // exists in the toRemoveEntities list.
  if (this->dataPtr->toRemoveEntities.find(_entity) !=
      this->dataPtr->toRemoveEntities.end())
  {
    // Find the entity in the message, and add if not present.
    entIter = _msg.mutable_entities()->find(_entity);
    if (entIter == _msg.mutable_entities()->end())
    {
      msgs::SerializedEntityMap ent;
      ent.set_id(_entity);
      (*_msg.mutable_entities())[static_cast<uint64_t>(_entity)] = ent;
      entIter = _msg.mutable_entities()->find(_entity);
    }

    entIter->second.set_remove(true);
  }

  // Insert all of the entity's components if the passed in types
  // set is empty
  auto types = _types;
  if (types.empty())
  {
    for (auto &type : this->dataPtr->entityComponents[_entity])
    {
      types.insert(type.first);
    }
  }

  // Empty means all types
  for (const ComponentTypeId type : types)
  {
    auto typeIter = iter->second.find(type);
    if (typeIter == iter->second.end())
    {
      continue;
    }

    const components::BaseComponent *compBase =
      this->ComponentImplementation(_entity, type);

    ComponentKey comp = {type, typeIter->second};

    // If not sending full state, skip unchanged components
    if (!_full &&
        this->dataPtr->oneTimeChangedComponents.find(comp) ==
        this->dataPtr->oneTimeChangedComponents.end() &&
        this->dataPtr->periodicChangedComponents.find(comp) ==
        this->dataPtr->periodicChangedComponents.end())
    {
      continue;
    }

    /// Find the entity in the message, if not already found.
    /// Add the entity to the message, if not already added.
    if (entIter == _msg.mutable_entities()->end())
    {
      entIter = _msg.mutable_entities()->find(_entity);
      if (entIter == _msg.mutable_entities()->end())
      {
        msgs::SerializedEntityMap ent;
        ent.set_id(_entity);
        (*_msg.mutable_entities())[static_cast<uint64_t>(_entity)] = ent;
        entIter = _msg.mutable_entities()->find(_entity);
      }
    }

    auto compIter = entIter->second.mutable_components()->find(comp.first);
    // Find the component in the message, and add the component to the
    // message if it's not present.
    if (compIter == entIter->second.mutable_components()->end())
    {
      msgs::SerializedComponent cmp;
      cmp.set_type(compBase->TypeId());
      (*(entIter->second.mutable_components()))[
        static_cast<int64_t>(comp.first)] = cmp;
      compIter = entIter->second.mutable_components()->find(comp.first);
    }

    // Serialize and store the message
    std::ostringstream ostr;
    compBase->Serialize(ostr);
    compIter->second.set_component(ostr.str());
  }

  // Add a component to the message and set it to be removed if the component
  // exists in the removedComponents map.
  this->dataPtr->SetRemovedComponentsMsgs(_entity, _msg, _types);
}

//////////////////////////////////////////////////
ignition::msgs::SerializedState EntityComponentManager::ChangedState() const
{
  ignition::msgs::SerializedState stateMsg;

  // New entities
  for (const auto &entity : this->dataPtr->newlyCreatedEntities)
  {
    this->AddEntityToMessage(stateMsg, entity);
  }

  // Entities being removed
  for (const auto &entity : this->dataPtr->toRemoveEntities)
  {
    this->AddEntityToMessage(stateMsg, entity);
  }

  // New / removed / changed components
  for (const auto &entity : this->dataPtr->modifiedComponents)
  {
    this->AddEntityToMessage(stateMsg, entity);
  }

  return stateMsg;
}

//////////////////////////////////////////////////
void EntityComponentManager::ChangedState(
    ignition::msgs::SerializedStateMap &_state) const
{
  // New entities
  for (const auto &entity : this->dataPtr->newlyCreatedEntities)
  {
    this->AddEntityToMessage(_state, entity);
  }

  // Entities being removed
  for (const auto &entity : this->dataPtr->toRemoveEntities)
  {
    this->AddEntityToMessage(_state, entity);
  }

  // New / removed / changed components
  for (const auto &entity : this->dataPtr->modifiedComponents)
  {
    this->AddEntityToMessage(_state, entity);
  }
}

//////////////////////////////////////////////////
void EntityComponentManagerPrivate::CalculateStateThreadLoad()
{
  // If the entity component vector is dirty, we need to recalculate the
  // threads and each threads work load
  if (!this->entityComponentsDirty)
    return;

  this->entityComponentsDirty = false;
  this->entityComponentIterators.clear();
  auto startIt = this->entityComponents.begin();
  int numComponents = this->entityComponents.size();

  // Set the number of threads to spawn to the min of the calculated thread
  // count or max threads that the hardware supports
  int maxThreads = std::thread::hardware_concurrency();
  uint64_t numThreads = std::min(numComponents, maxThreads);

  int componentsPerThread = static_cast<int>(std::ceil(
    static_cast<double>(numComponents) / numThreads));

  igndbg << "Updated state thread iterators: " << numThreads
         << " threads processing around " << componentsPerThread
         << " components each." << std::endl;

  // Push back the starting iterator
  this->entityComponentIterators.push_back(startIt);
  for (uint64_t i = 0; i < numThreads; ++i)
  {
    // If we have added all of the components to the iterator vector, we are
    // done so push back the end iterator
    numComponents -= componentsPerThread;
    if (numComponents <= 0)
    {
      this->entityComponentIterators.push_back(
          this->entityComponents.end());
      break;
    }

    // Get the iterator to the next starting group of components
    auto nextIt = std::next(startIt, componentsPerThread);
    this->entityComponentIterators.push_back(nextIt);
    startIt = nextIt;
  }
}

//////////////////////////////////////////////////
ignition::msgs::SerializedState EntityComponentManager::State(
    const std::unordered_set<Entity> &_entities,
    const std::unordered_set<ComponentTypeId> &_types) const
{
  ignition::msgs::SerializedState stateMsg;
  for (const auto &it : this->dataPtr->entityComponents)
  {
    auto entity = it.first;
    if (!_entities.empty() && _entities.find(entity) == _entities.end())
    {
      continue;
    }

    this->AddEntityToMessage(stateMsg, entity, _types);
  }

  return stateMsg;
}

//////////////////////////////////////////////////
void EntityComponentManager::State(
    msgs::SerializedStateMap  &_state,
    const std::unordered_set<Entity> &_entities,
    const std::unordered_set<ComponentTypeId> &_types,
    bool _full) const
{
  std::mutex stateMapMutex;
  std::vector<std::thread> workers;

  this->dataPtr->CalculateStateThreadLoad();

  auto functor = [&](auto itStart, auto itEnd)
  {
    msgs::SerializedStateMap threadMap;
    while (itStart != itEnd)
    {
      auto entity = (*itStart).first;
      if (_entities.empty() || _entities.find(entity) != _entities.end())
      {
        this->AddEntityToMessage(threadMap, entity, _types, _full);
      }
      itStart++;
    }
    std::lock_guard<std::mutex> lock(stateMapMutex);

    for (const auto &entity : threadMap.entities())
    {
      (*_state.mutable_entities())[static_cast<uint64_t>(entity.first)] =
          entity.second;
    }
  };

  // Spawn workers
  uint64_t numThreads = this->dataPtr->entityComponentIterators.size() - 1;
  for (uint64_t i = 0; i < numThreads; i++)
  {
    workers.push_back(std::thread(functor,
        this->dataPtr->entityComponentIterators[i],
        this->dataPtr->entityComponentIterators[i+1]));
  }

  // Wait for each thread to finish processing its components
  std::for_each(workers.begin(), workers.end(), [](std::thread &_t)
  {
    _t.join();
  });
}

//////////////////////////////////////////////////
void EntityComponentManager::SetState(
    const ignition::msgs::SerializedState &_stateMsg)
{
  IGN_PROFILE("EntityComponentManager::SetState Non-map");
  // Create / remove / update entities
  for (int e = 0; e < _stateMsg.entities_size(); ++e)
  {
    const auto &entityMsg = _stateMsg.entities(e);

    Entity entity{entityMsg.id()};

    // Remove entity
    if (entityMsg.remove())
    {
      this->RequestRemoveEntity(entity);
      continue;
    }

    // Create entity if it doesn't exist
    if (!this->HasEntity(entity))
    {
      this->dataPtr->CreateEntityImplementation(entity);
    }

    // Create / remove / update components
    for (int c = 0; c < entityMsg.components_size(); ++c)
    {
      const auto &compMsg = entityMsg.components(c);

      // Skip if component not set. Note that this will also skip components
      // setting an empty value.
      if (compMsg.component().empty())
      {
        continue;
      }

      auto type = compMsg.type();

      // Components which haven't been registered in this process, such as 3rd
      // party components streamed to other secondaries and the GUI.
      if (!components::Factory::Instance()->HasType(type))
      {
        static std::unordered_set<unsigned int> printedComps;
        if (printedComps.find(type) == printedComps.end())
        {
          printedComps.insert(type);
          ignwarn << "Component type [" << type << "] has not been "
                  << "registered in this process, so it can't be deserialized."
                  << std::endl;
        }
        continue;
      }

      // Create component
      auto newComp = components::Factory::Instance()->New(compMsg.type());

      if (nullptr == newComp)
      {
        ignerr << "Failed to deserialize component of type [" << compMsg.type()
               << "]" << std::endl;
        continue;
      }

      std::istringstream istr(compMsg.component());
      newComp->Deserialize(istr);

      // Get type id
      auto typeId = newComp->TypeId();

      // TODO(louise) Move into if, see TODO below
      this->RemoveComponent(entity, typeId);

      // Remove component
      if (compMsg.remove())
      {
        continue;
      }

      // Get Component
      auto comp = this->ComponentImplementation(entity, typeId);

      // Create if new
      if (nullptr == comp)
      {
        this->CreateComponentImplementation(entity, typeId, newComp.get());
      }
      // Update component value
      else
      {
        ignerr << "Internal error" << std::endl;
        // TODO(louise) We're shortcutting above and always  removing the
        // component so that we don't get here, gotta figure out why this
        // doesn't update the component. The following line prints the correct
        // values.
        // igndbg << *comp << "  " << *newComp.get() << std::endl;
        // *comp = *newComp.get();

        // When above TODO is addressed, uncomment AddModifiedComponent below
        // unless calling SetChanged (which already calls AddModifiedComponent)
        // this->dataPtr->AddModifiedComponent(entity);
      }
    }
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::SetState(
    const ignition::msgs::SerializedStateMap &_stateMsg)
{
  IGN_PROFILE("EntityComponentManager::SetState Map");
  // Create / remove / update entities
  for (const auto &iter : _stateMsg.entities())
  {
    const auto &entityMsg = iter.second;

    Entity entity{entityMsg.id()};

    // Remove entity
    if (entityMsg.remove())
    {
      this->RequestRemoveEntity(entity);
      continue;
    }

    // Create entity if it doesn't exist
    if (!this->HasEntity(entity))
    {
      this->dataPtr->CreateEntityImplementation(entity);
    }

    // Create / remove / update components
    for (const auto &compIter : iter.second.components())
    {
      const auto &compMsg = compIter.second;

      // Skip if component not set. Note that this will also skip components
      // setting an empty value.
      if (compMsg.component().empty())
      {
        continue;
      }

      uint64_t type = compMsg.type();

      // Components which haven't been registered in this process, such as 3rd
      // party components streamed to other secondaries and the GUI.
      if (!components::Factory::Instance()->HasType(type))
      {
        static std::unordered_set<unsigned int> printedComps;
        if (printedComps.find(type) == printedComps.end())
        {
          printedComps.insert(type);
          ignwarn << "Component type [" << type << "] has not been "
                  << "registered in this process, so it can't be deserialized."
                  << std::endl;
        }
        continue;
      }

      // Remove component
      if (compMsg.remove())
      {
        this->RemoveComponent(entity, compIter.first);
        continue;
      }

      // Get Component
      components::BaseComponent *comp =
        this->ComponentImplementation(entity, compIter.first);

      // Create if new
      if (nullptr == comp)
      {
        // Create component
        auto newComp = components::Factory::Instance()->New(compMsg.type());

        if (nullptr == newComp)
        {
          ignerr << "Failed to create component of type [" << compMsg.type()
            << "]" << std::endl;
          continue;
        }

        std::istringstream istr(compMsg.component());
        newComp->Deserialize(istr);

        this->CreateComponentImplementation(entity,
            newComp->TypeId(), newComp.get());
      }
      // Update component value
      else
      {
        std::istringstream istr(compMsg.component());
        comp->Deserialize(istr);
        this->SetChanged(entity, compIter.first,
            _stateMsg.has_one_time_component_changes() ?
            ComponentState::OneTimeChange :
            ComponentState::PeriodicChange);
      }
    }
  }
}

//////////////////////////////////////////////////
std::unordered_set<Entity> EntityComponentManager::Descendants(Entity _entity)
    const
{
  // Check cache
  if (this->dataPtr->descendantCache.find(_entity) !=
      this->dataPtr->descendantCache.end())
  {
    return this->dataPtr->descendantCache[_entity];
  }

  std::unordered_set<Entity> descendants;

  if (!this->HasEntity(_entity))
    return descendants;

  auto descVector = math::graph::BreadthFirstSort(this->dataPtr->entities,
      _entity);
  std::move(descVector.begin(), descVector.end(), std::inserter(descendants,
      descendants.end()));

  this->dataPtr->descendantCache[_entity] = descendants;
  return descendants;
}

//////////////////////////////////////////////////
void EntityComponentManager::SetAllComponentsUnchanged()
{
  this->dataPtr->periodicChangedComponents.clear();
  this->dataPtr->oneTimeChangedComponents.clear();
  this->dataPtr->modifiedComponents.clear();
}

/////////////////////////////////////////////////
void EntityComponentManager::SetChanged(
    const Entity _entity, const ComponentTypeId _type,
    gazebo::ComponentState _c)
{
  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return;

  auto typeIter = ecIter->second.find(_type);
  if (typeIter == ecIter->second.end())
    return;

  ComponentKey key{_type, typeIter->second};

  if (_c == ComponentState::PeriodicChange)
  {
    this->dataPtr->periodicChangedComponents.insert(key);
    this->dataPtr->oneTimeChangedComponents.erase(key);
  }
  else if (_c == ComponentState::OneTimeChange)
  {
    this->dataPtr->periodicChangedComponents.erase(key);
    this->dataPtr->oneTimeChangedComponents.insert(key);
  }
  else
  {
    this->dataPtr->periodicChangedComponents.erase(key);
    this->dataPtr->oneTimeChangedComponents.erase(key);
  }

  this->dataPtr->AddModifiedComponent(_entity);
}

/////////////////////////////////////////////////
std::unordered_set<ComponentTypeId> EntityComponentManager::ComponentTypes(
    const Entity _entity) const
{
  std::unordered_set<ComponentTypeId> result;

  auto it = this->dataPtr->entityComponents.find(_entity);
  if (it == this->dataPtr->entityComponents.end())
    return result;

  for (const auto &key : it->second)
  {
    result.insert(key.first);
  }

  return result;
}

/////////////////////////////////////////////////
void EntityComponentManager::SetEntityCreateOffset(uint64_t _offset)
{
  if (_offset < this->dataPtr->entityCount)
  {
    ignwarn << "Setting an entity offset of [" << _offset << "] is less than "
     << "the current entity count of [" << this->dataPtr->entityCount << "]. "
     << "Incorrect behavior should be expected.\n";
  }

  this->dataPtr->entityCount = _offset;
}

/////////////////////////////////////////////////
void EntityComponentManagerPrivate::AddModifiedComponent(const Entity &_entity)
{
  if (this->newlyCreatedEntities.find(_entity)
        != this->newlyCreatedEntities.end() ||
      this->toRemoveEntities.find(_entity) != this->toRemoveEntities.end() ||
      this->modifiedComponents.find(_entity) != this->modifiedComponents.end())
  {
    // modified component is already in newlyCreatedEntities
    // or toRemoveEntities list
    return;
  }

  this->modifiedComponents.insert(_entity);
}
