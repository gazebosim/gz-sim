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

#include "gz/sim/EntityComponentManager.hh"
#include "EntityComponentManagerDiff.hh"

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/graph/GraphAlgorithms.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Component.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Recreate.hh"
#include "gz/sim/components/World.hh"

using namespace gz;
using namespace sim;

class gz::sim::EntityComponentManagerPrivate
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

  /// \brief Recursively erase an entity and all its descendants from a given
  /// set.
  /// \param[in] _entity Entity to be erased.
  /// \param[in, out] _set Set to erase from.
  public: void EraseEntityRecursive(Entity _entity,
      std::unordered_set<Entity> &_set);

  /// \brief Allots the work for multiple threads prior to running
  /// `AddEntityToMessage`.
  public: void CalculateStateThreadLoad();

  /// \brief Copies the contents of `_from` into this object.
  /// \note This is a member function instead of a copy constructor so that
  /// it can have additional parameters if the need arises in the future.
  /// Additionally, not every data member is copied making its behavior
  /// different from what would be expected from a copy constructor.
  /// \param[in] _from Object to copy from
  public: void CopyFrom(const EntityComponentManagerPrivate &_from);

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

  /// \brief Check whether a component is marked as a component that is
  /// currently removed or not.
  /// \param[in] _entity The entity
  /// \param[in] _typeId The type ID for the component that belongs to _entity
  /// \return True if _entity has a component of type _typeId that is currently
  /// removed. False otherwise
  public: bool ComponentMarkedAsRemoved(const Entity _entity,
              const ComponentTypeId _typeId) const;

  /// \brief Set a cloned joint's parent or child link name.
  /// \param[in] _joint The cloned joint.
  /// \param[in] _originalLink The original joint's parent or child link.
  /// \param[in] _ecm Entity component manager.
  /// \tparam The component type, which must be either
  /// components::ParentLinkName or components::ChildLinkName
  /// \return True if _joint's parent or child link name was set.
  /// False otherwise
  /// \note This method should only be called in EntityComponentManager::Clone.
  /// This is a temporary workaround until we find a way to clone entites and
  /// components that don't require special treatment for particular component
  /// types.
  public: template<typename ComponentTypeT>
          bool ClonedJointLinkName(Entity _joint, Entity _originalLink,
              EntityComponentManager *_ecm);

  /// \brief All component types that have ever been created.
  public: std::unordered_set<ComponentTypeId> createdCompTypes;

  /// \brief A graph holding all entities, arranged according to their
  /// parenting.
  public: EntityGraph entities;

  /// \brief Components that have been changed through a periodic change.
  /// The key is the type of component which has changed, and the value is the
  /// entities that had this type of component changed.
  public: std::unordered_map<ComponentTypeId, std::unordered_set<Entity>>
            periodicChangedComponents;

  /// \brief Components that have been changed through a one-time change.
  /// The key is the type of component which has changed, and the value is the
  /// entities that had this type of component changed.
  public: std::unordered_map<ComponentTypeId, std::unordered_set<Entity>>
            oneTimeChangedComponents;

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

  /// \brief A mutex to protect newly created entities.
  public: std::mutex entityCreatedMutex;

  /// \brief A mutex to protect entity remove.
  public: std::mutex entityRemoveMutex;

  /// \brief A mutex to protect from concurrent writes to views
  public: mutable std::mutex viewsMutex;

  /// \brief A mutex to protect removed components
  public: mutable std::mutex removedComponentsMutex;

  /// \brief The set of all views.
  /// The value is a pair of the view itself and a mutex that can be used for
  /// locking the view to ensure thread safety when adding entities to the view.
  public: mutable std::unordered_map<detail::ComponentTypeKey,
          std::pair<std::unique_ptr<detail::BaseView>,
            std::unique_ptr<std::mutex>>, detail::ComponentTypeHasher> views;

  /// \brief A flag that indicates whether views should be locked while adding
  /// new entities to them or not.
  public: bool lockAddEntitiesToViews{false};

  /// \brief Cache of previously queried descendants. The key is the parent
  /// entity for which descendants were queried, and the value are all its
  /// descendants.
  public: mutable std::unordered_map<Entity, std::unordered_set<Entity>>
          descendantCache;

  /// \brief Keep track of entities already used to ensure uniqueness.
  public: uint64_t entityCount{0};

  /// \brief Unordered map of removed components. The key is the entity to
  /// which belongs the component, and the value is a set of the component types
  /// being removed.
  public: std::unordered_map<Entity, std::unordered_set<ComponentTypeId>>
    removedComponents;

  /// \brief All components that have been removed. The difference between
  /// removedComponents and componentsMarkedAsRemoved is that removedComponents
  /// keeps track of components that were removed in the current simulation
  /// step, while componentsMarkedAsRemoved keeps track of components that are
  /// currently removed based on all simulation steps.
  public: std::unordered_map<Entity, std::unordered_set<ComponentTypeId>>
    componentsMarkedAsRemoved;

  /// \brief A map of an entity to its components
  public: std::unordered_map<Entity,
           std::vector<std::unique_ptr<components::BaseComponent>>>
             componentStorage;

  /// \brief A map that keeps track of where each type of component is
  /// located in the componentStorage vector. Since the componentStorage vector
  /// is of type BaseComponent, we need to keep track of which component type
  /// corresponds to a given index in the vector so that we can cast the
  /// BaseComponent to this type if needed.
  ///
  /// The key of this map is the Entity, and the value is a map of the
  /// component type to the corresponding index in the
  /// componentStorage vector (a component of a particular type is
  /// only a key for the value map if a component of this type exists in
  /// the componentStorage vector)
  ///
  /// NOTE: Any modification of this data structure must be followed
  /// by setting `componentTypeIndexDirty` to true.
  public: std::unordered_map<Entity,
           std::unordered_map<ComponentTypeId, std::size_t>>
                                componentTypeIndex;

  /// \brief A vector of iterators to evenly distributed spots in the
  /// `componentTypeIndex` map.  Threads in the `State` function use this
  /// vector for easy access of their pre-allocated work.  This vector
  /// is recalculated if `componentTypeIndex` is changed (when
  /// `componentTypeIndexDirty` == true).
  public: std::vector<std::unordered_map<Entity,
          std::unordered_map<ComponentTypeId, std::size_t>>::iterator>
            componentTypeIndexIterators;

  /// \brief True if the componentTypeIndex map was changed.  Primarily used
  /// by the multithreading functionality in `State()` to allocate work to
  /// each thread.
  public: bool componentTypeIndexDirty{true};

  /// \brief During cloning, we populate two maps:
  ///  - map of cloned model entities to the non-cloned model's canonical link
  ///  - map of non-cloned canonical links to the cloned canonical link
  /// After cloning is done, these maps can be used to update the cloned model's
  /// canonical link to be the cloned canonical link instead of the original
  /// model's canonical link. We populate maps during cloning and then update
  /// canonical links after cloning since cloning is done top-down, and
  /// canonical links are children of models (when a model is cloned, its
  /// canonical link has not been cloned yet, so we have no way of knowing what
  /// to set the cloned model's canonical link to until the canonical link has
  /// been cloned).
  /// \TODO(anyone) We shouldn't be giving canonical links special treatment.
  /// This may happen to any component that holds an Entity, so we should figure
  /// out a way to generalize this for any such component.
  public: std::unordered_map<Entity, Entity> oldModelCanonicalLink;

  /// \brief See above
  public: std::unordered_map<Entity, Entity> oldToClonedCanonicalLink;

  /// \brief During cloning, we populate two maps:
  ///  - map of link entities to their cloned link
  ///  - map of cloned joint entities to the original joint entity's parent and
  ///    child links
  /// After cloning is done, these maps can be used to update the cloned joint
  /// entity's parent and child links to the cloned parent and child links.
  /// \TODO(anyone) We shouldn't be giving joints special treatment.
  /// We should figure out a way to update a joint's parent/child links without
  /// having to explicitly search/track for the cloned links.
  public: std::unordered_map<Entity, Entity> originalToClonedLink;

  /// \brief See above
  /// The key is the cloned joint entity, and the value is a pair where the
  /// first element is the original joint's parent link, and the second element
  /// is the original joint's child link
  public: std::unordered_map<Entity, std::pair<Entity, Entity>>
          clonedToOriginalJointLinks;

  /// \brief Set of entities that are prevented from removal.
  public: std::unordered_set<Entity> pinnedEntities;
};

//////////////////////////////////////////////////
EntityComponentManager::EntityComponentManager()
  : dataPtr(new EntityComponentManagerPrivate)
{
}

//////////////////////////////////////////////////
EntityComponentManager::~EntityComponentManager() = default;

//////////////////////////////////////////////////
void EntityComponentManagerPrivate::CopyFrom(
    const EntityComponentManagerPrivate &_from)
{
  this->createdCompTypes = _from.createdCompTypes;
  this->entities = _from.entities;
  this->periodicChangedComponents = _from.periodicChangedComponents;
  this->oneTimeChangedComponents = _from.oneTimeChangedComponents;
  this->newlyCreatedEntities = _from.newlyCreatedEntities;
  this->toRemoveEntities = _from.toRemoveEntities;
  this->modifiedComponents = _from.modifiedComponents;
  this->removeAllEntities = _from.removeAllEntities;
  this->views.clear();
  this->lockAddEntitiesToViews = _from.lockAddEntitiesToViews;
  this->descendantCache.clear();
  this->entityCount = _from.entityCount;
  this->removedComponents = _from.removedComponents;
  this->componentsMarkedAsRemoved = _from.componentsMarkedAsRemoved;

  for (const auto &[entity, comps] : _from.componentStorage)
  {
    this->componentStorage[entity].clear();
    for (const auto &comp : comps)
    {
      this->componentStorage[entity].emplace_back(comp->Clone());
    }
  }
  this->componentTypeIndex = _from.componentTypeIndex;
  this->componentTypeIndexIterators.clear();
  this->componentTypeIndexDirty = true;

  // Not copying maps related to cloning since they are transient variables
  // that are used as return values of some member functions.

  this->pinnedEntities = _from.pinnedEntities;
}

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
    gzwarn << "Reached maximum number of entities [" << entity << "]"
            << std::endl;
    return entity;
  }

  return this->dataPtr->CreateEntityImplementation(entity);
}

/////////////////////////////////////////////////
Entity EntityComponentManagerPrivate::CreateEntityImplementation(Entity _entity)
{
  GZ_PROFILE("EntityComponentManager::CreateEntityImplementation");
  this->entities.AddVertex(std::to_string(_entity), _entity, _entity);

  // Add entity to the list of newly created entities
  {
    std::lock_guard<std::mutex> lock(this->entityCreatedMutex);
    this->newlyCreatedEntities.insert(_entity);
  }

  // Reset descendants cache
  this->descendantCache.clear();

  const auto result = this->componentStorage.insert({_entity,
      std::vector<std::unique_ptr<components::BaseComponent>>()});
  if (!result.second)
  {
    gzwarn << "Attempted to add entity [" << _entity
      << "] to component storage, but this entity is already in component "
      << "storage.\n";
  }

  const auto result2 = this->componentTypeIndex.insert({_entity,
      std::unordered_map<ComponentTypeId, std::size_t>()});
  if (!result2.second)
  {
    gzwarn << "Attempted to add entity [" << _entity
      << "] to component type index, but this entity is already in component "
      << "type index.\n";
  }

  return _entity;
}

/////////////////////////////////////////////////
Entity EntityComponentManager::Clone(Entity _entity, Entity _parent,
    const std::string &_name, bool _allowRename)
{
  // Clear maps so they're populated for the entity being cloned
  this->dataPtr->oldToClonedCanonicalLink.clear();
  this->dataPtr->oldModelCanonicalLink.clear();
  this->dataPtr->originalToClonedLink.clear();
  this->dataPtr->clonedToOriginalJointLinks.clear();

  auto clonedEntity = this->CloneImpl(_entity, _parent, _name, _allowRename);

  if (kNullEntity != clonedEntity)
  {
    // make sure that cloned models have their canonical link updated to the
    // cloned canonical link
    for (const auto &[clonedModel, oldCanonicalLink] :
         this->dataPtr->oldModelCanonicalLink)
    {
      auto iter = this->dataPtr->oldToClonedCanonicalLink.find(
          oldCanonicalLink);
      if (iter == this->dataPtr->oldToClonedCanonicalLink.end())
      {
        gzerr << "Error: attempted to clone model(s) with canonical link(s), "
          << "but entity [" << oldCanonicalLink << "] was not cloned as a "
          << "canonical link." << std::endl;
        continue;
      }
      const auto clonedCanonicalLink = iter->second;
      this->SetComponentData<components::ModelCanonicalLink>(clonedModel,
          clonedCanonicalLink);
    }

    // make sure that cloned joints have their parent/child links
    // updated to the cloned parent/child links
    for (const auto &[clonedJoint, originalJointLinks] :
        this->dataPtr->clonedToOriginalJointLinks)
    {
      auto originalParentLink = originalJointLinks.first;
      if (!this->dataPtr->ClonedJointLinkName<components::ParentLinkName>(
            clonedJoint, originalParentLink, this))
      {
        gzerr << "Error updating the cloned parent link name for cloned "
               << "joint [" << clonedJoint << "]\n";
        continue;
      }

      auto originalChildLink = originalJointLinks.second;
      if (!this->dataPtr->ClonedJointLinkName<components::ChildLinkName>(
            clonedJoint, originalChildLink, this))
      {
        gzerr << "Error updating the cloned child link name for cloned "
               << "joint [" << clonedJoint << "]\n";
        continue;
      }
    }
  }

  return clonedEntity;
}

/////////////////////////////////////////////////
Entity EntityComponentManager::CloneImpl(Entity _entity, Entity _parent,
    const std::string &_name, bool _allowRename)
{
  auto uniqueNameGenerated = false;

  // Before cloning, we should make sure that:
  //  1. The entity to be cloned exists
  //  2. We can generate a unique name for the cloned entity
  if (!this->HasEntity(_entity))
  {
    gzerr << "Requested to clone entity [" << _entity
      << "], but this entity does not exist." << std::endl;
    return kNullEntity;
  }
  else if (!_name.empty() && !_allowRename)
  {
    // Get the entity's original parent. This is used to make sure we get
    // the correct entity. For example, two different models may have a
    // child with the name "link".
    auto origParentComp =
        this->Component<components::ParentEntity>(_entity);

    // If there is an entity with the same name and user indicated renaming is
    // not allowed then return null entity.
    // If the entity or one of its ancestor has a Recreate component then carry
    // on since the ECM is supposed to create a new entity with the same name.
    Entity ent = this->EntityByComponents(components::Name(_name),
        components::ParentEntity(origParentComp->Data()));

    bool hasRecreateComp = false;
    Entity recreateEnt = ent;
    while (recreateEnt != kNullEntity && !hasRecreateComp)
    {
      hasRecreateComp = this->Component<components::Recreate>(recreateEnt) !=
        nullptr;
      auto parentComp = this->Component<components::ParentEntity>(recreateEnt);
      recreateEnt = parentComp ? parentComp->Data() : kNullEntity;
    }

    if (kNullEntity != ent && !hasRecreateComp)
    {
      gzerr << "Requested to clone entity [" << _entity
        << "] with a name of [" << _name << "], but another entity already "
        << "has this name." << std::endl;
      return kNullEntity;
    }
    uniqueNameGenerated = true;
  }

  auto clonedEntity = this->CreateEntity();

  if (_parent != kNullEntity)
  {
    this->SetParentEntity(clonedEntity, _parent);
    this->CreateComponent(clonedEntity, components::ParentEntity(_parent));
  }

  // make sure that the cloned entity has a unique name
  auto clonedName = _name;
  if (!uniqueNameGenerated)
  {
    if (clonedName.empty())
    {
      auto originalNameComp = this->Component<components::Name>(_entity);
      clonedName =
        originalNameComp ? originalNameComp->Data() : "cloned_entity";
    }
    uint64_t suffix = 1;
    while (kNullEntity != this->EntityByComponents(
          components::Name(clonedName + "_" + std::to_string(suffix))))
      suffix++;
    clonedName += "_" + std::to_string(suffix);
  }
  this->CreateComponent(clonedEntity, components::Name(clonedName));

  // copy all components from _entity to clonedEntity
  for (const auto &type : this->ComponentTypes(_entity))
  {
    // skip the Name and ParentEntity components since those were already
    // handled above
    if ((type == components::Name::typeId) ||
        (type == components::ParentEntity::typeId))
      continue;

    auto originalComp = this->ComponentImplementation(_entity, type);
    auto clonedComp = originalComp->Clone();

    auto updateData =
      this->CreateComponentImplementation(clonedEntity, type, clonedComp.get());
    if (updateData)
    {
      // When a cloned entity is removed, it erases all components/data so a new
      // cloned entity should not have components to be updated
      // LCOV_EXCL_START
      gzerr << "Internal error: The component's data needs to be updated but "
             << "this should not happen." << std::endl;
      // LCOV_EXCL_STOP
    }
  }

  // keep track of canonical link information (for clones of models, the cloned
  // model should not share the same canonical link as the original model)
  if (auto modelCanonLinkComp =
      this->Component<components::ModelCanonicalLink>(clonedEntity))
  {
    // we're cloning a model, so we map the cloned model to the original
    // model's canonical link
    this->dataPtr->oldModelCanonicalLink[clonedEntity] =
        modelCanonLinkComp->Data();
  }
  else if (this->Component<components::CanonicalLink>(clonedEntity))
  {
    // we're cloning a canonical link, so we map the original canonical link
    // to the cloned canonical link
    this->dataPtr->oldToClonedCanonicalLink[_entity] = clonedEntity;
  }

  // keep track of all joints and links that have been cloned so that cloned
  // joints can be updated to their cloned parent/child links
  if (this->Component<components::Joint>(clonedEntity))
  {
    // this is a joint, so we need to find the original joint's parent and child
    // link entities
    Entity originalParentLink = kNullEntity;
    Entity originalChildLink = kNullEntity;

    auto origParentComp =
        this->Component<components::ParentEntity>(_entity);

    const auto &parentName =
      this->Component<components::ParentLinkName>(_entity);
    if (parentName && origParentComp)
    {
      // Handle the case where the parent link name is the world.
      if (common::lowercase(parentName->Data()) == "world")
      {
        originalParentLink = this->Component<components::ParentEntity>(
            origParentComp->Data())->Data();
      }
      else
      {
        originalParentLink =
          this->EntityByComponents<components::Name, components::ParentEntity>(
              components::Name(parentName->Data()),
              components::ParentEntity(origParentComp->Data()));
      }
    }

    const auto &childName = this->Component<components::ChildLinkName>(_entity);
    if (childName && origParentComp)
    {
      originalChildLink =
        this->EntityByComponents<components::Name, components::ParentEntity>(
          components::Name(childName->Data()),
          components::ParentEntity(origParentComp->Data()));
    }

    if (!originalParentLink || !originalChildLink)
    {
      gzerr << "The cloned joint entity [" << clonedEntity << "] was unable "
        << "to find the original joint entity's parent and/or child link.\n";
      this->RequestRemoveEntity(clonedEntity);
      return kNullEntity;
    }

    this->dataPtr->clonedToOriginalJointLinks[clonedEntity] =
      {originalParentLink, originalChildLink};
  }
  else if (this->Component<components::Link>(clonedEntity) ||
      this->Component<components::CanonicalLink>(clonedEntity))
  {
    // save a mapping between the original link and the cloned link
    this->dataPtr->originalToClonedLink[_entity] = clonedEntity;
  }

  for (const auto &childEntity :
      this->EntitiesByComponents(components::ParentEntity(_entity)))
  {
    std::string name;
    if (!_allowRename)
    {
      auto nameComp = this->Component<components::Name>(childEntity);
      name = nameComp->Data();
    }
    auto clonedChild = this->CloneImpl(childEntity, clonedEntity, name,
        _allowRename);
    if (kNullEntity == clonedChild)
    {
      gzerr << "Cloning child entity [" << childEntity << "] failed.\n";
      this->RequestRemoveEntity(clonedEntity);
      return kNullEntity;
    }
  }

  return clonedEntity;
}

/////////////////////////////////////////////////
void EntityComponentManager::ClearNewlyCreatedEntities()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
  this->dataPtr->newlyCreatedEntities.clear();

  for (auto &view : this->dataPtr->views)
  {
    view.second.first->ResetNewEntityState();
  }
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasRemovedComponents() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->removedComponentsMutex);
  return !this->dataPtr->removedComponents.empty();
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
void EntityComponentManagerPrivate::EraseEntityRecursive(Entity _entity,
    std::unordered_set<Entity> &_set)
{
  for (const auto &vertex : this->entities.AdjacentsFrom(_entity))
  {
    this->EraseEntityRecursive(vertex.first, _set);
  }
  _set.erase(_entity);
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

  // Remove entities from tmpToRemoveEntities that are marked as
  // unremovable.
  for (auto iter = tmpToRemoveEntities.begin();
       iter != tmpToRemoveEntities.end();)
  {
    if (std::find(this->dataPtr->pinnedEntities.begin(),
                  this->dataPtr->pinnedEntities.end(), *iter) !=
               this->dataPtr->pinnedEntities.end())
    {
      iter = tmpToRemoveEntities.erase(iter);
    }
    else
    {
      ++iter;
    }
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
    this->dataPtr->toRemoveEntities.insert(tmpToRemoveEntities.begin(),
                                          tmpToRemoveEntities.end());
  }

  for (const auto &removedEntity : tmpToRemoveEntities)
  {
    for (auto &view : this->dataPtr->views)
    {
      view.second.first->MarkEntityToRemove(removedEntity);
    }
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestRemoveEntities()
{
  if (this->dataPtr->pinnedEntities.empty())
  {
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
      this->dataPtr->removeAllEntities = true;
    }
    this->RebuildViews();
  }
  else
  {
    std::unordered_set<Entity> tmpToRemoveEntities;

    // Store the to-be-removed entities in a temporary set so we can
    // mark each of them to be removed from views that contain them.
    for (const auto &vertex : this->dataPtr->entities.Vertices())
    {
      if (std::find(this->dataPtr->pinnedEntities.begin(),
                    this->dataPtr->pinnedEntities.end(), vertex.first) ==
          this->dataPtr->pinnedEntities.end())
      {
        tmpToRemoveEntities.insert(vertex.first);
      }
    }

    {
      std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
      this->dataPtr->toRemoveEntities.insert(tmpToRemoveEntities.begin(),
          tmpToRemoveEntities.end());
    }

    for (const auto &removedEntity : tmpToRemoveEntities)
    {
      for (auto &view : this->dataPtr->views)
      {
        view.second.first->MarkEntityToRemove(removedEntity);
      }
    }
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::ProcessRemoveEntityRequests()
{
  GZ_PROFILE("EntityComponentManager::ProcessRemoveEntityRequests");
  std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
  // Short-cut if erasing all entities
  if (this->dataPtr->removeAllEntities)
  {
    GZ_PROFILE("RemoveAll");
    this->dataPtr->removeAllEntities = false;
    this->dataPtr->entities = EntityGraph();
    this->dataPtr->toRemoveEntities.clear();
    this->dataPtr->componentsMarkedAsRemoved.clear();

    // reset the entity component storage
    this->dataPtr->componentStorage.clear();
    this->dataPtr->componentTypeIndex.clear();
    this->dataPtr->componentTypeIndexDirty = true;

    // All views are now invalid.
    this->dataPtr->views.clear();
  }
  else
  {
    GZ_PROFILE("Remove");
    // Otherwise iterate through the list of entities to remove.
    for (const Entity entity : this->dataPtr->toRemoveEntities)
    {
      // Make sure the entity exists and is not removed.
      if (!this->HasEntity(entity))
        continue;

      // Remove from graph
      this->dataPtr->entities.RemoveVertex(entity);

      this->dataPtr->componentsMarkedAsRemoved.erase(entity);
      this->dataPtr->componentStorage.erase(entity);
      this->dataPtr->componentTypeIndex.erase(entity);
      this->dataPtr->componentTypeIndexDirty = true;

      // Remove the entity from views.
      for (auto &view : this->dataPtr->views)
      {
        view.second.first->RemoveEntity(entity);
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
  GZ_PROFILE("EntityComponentManager::RemoveComponent");
  // Make sure the entity exists and has the component.
  if (!this->EntityHasComponentType(_entity, _typeId))
    return false;

  auto oneTimeIter = this->dataPtr->oneTimeChangedComponents.find(_typeId);
  if (oneTimeIter != this->dataPtr->oneTimeChangedComponents.end())
  {
    oneTimeIter->second.erase(_entity);
    if (oneTimeIter->second.empty())
      this->dataPtr->oneTimeChangedComponents.erase(oneTimeIter);
  }

  auto periodicIter = this->dataPtr->periodicChangedComponents.find(_typeId);
  if (periodicIter != this->dataPtr->periodicChangedComponents.end())
  {
    periodicIter->second.erase(_entity);
    if (periodicIter->second.empty())
      this->dataPtr->periodicChangedComponents.erase(periodicIter);
  }

  auto compPtr = this->ComponentImplementation(_entity, _typeId);
  if (compPtr)
  {
    this->dataPtr->componentsMarkedAsRemoved[_entity].insert(_typeId);

    // update views to reflect the component removal
    for (auto &viewPair : this->dataPtr->views)
      viewPair.second.first->NotifyComponentRemoval(_entity, _typeId);
  }

  this->dataPtr->AddModifiedComponent(_entity);

  // Add component to map of removed components
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->removedComponentsMutex);
    this->dataPtr->removedComponents[_entity].insert(_typeId);
  }

  return true;
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponentType(const Entity _entity,
    const ComponentTypeId &_typeId) const
{
  if (!this->HasEntity(_entity))
    return false;

  auto comp = this->ComponentImplementation(_entity, _typeId);

  return comp != nullptr;
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

  auto ctIter = this->dataPtr->componentTypeIndex.find(_entity);

  if (ctIter == this->dataPtr->componentTypeIndex.end())
    return result;

  auto typeIter = ctIter->second.find(_typeId);
  if (typeIter == ctIter->second.end() ||
      this->dataPtr->ComponentMarkedAsRemoved(_entity, _typeId))
    return result;

  auto typeId = typeIter->first;

  auto oneTimeIter = this->dataPtr->oneTimeChangedComponents.find(typeId);
  if (oneTimeIter != this->dataPtr->oneTimeChangedComponents.end() &&
      oneTimeIter->second.find(_entity) != oneTimeIter->second.end())
  {
    result = ComponentState::OneTimeChange;
  }
  else
  {
    auto periodicIter =
      this->dataPtr->periodicChangedComponents.find(typeId);
    if (periodicIter != this->dataPtr->periodicChangedComponents.end() &&
        periodicIter->second.find(_entity) != periodicIter->second.end())
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
bool EntityComponentManager::HasPeriodicComponentChanges() const
{
  return !this->dataPtr->periodicChangedComponents.empty();
}

/////////////////////////////////////////////////
std::unordered_set<ComponentTypeId>
    EntityComponentManager::ComponentTypesWithPeriodicChanges() const
{
  std::unordered_set<ComponentTypeId> periodicComponents;
  for (const auto& typeToEntityPtrs : this->dataPtr->periodicChangedComponents)
  {
    periodicComponents.insert(typeToEntityPtrs.first);
  }
  return periodicComponents;
}

/////////////////////////////////////////////////
void EntityComponentManager::UpdatePeriodicChangeCache(
  std::unordered_map<ComponentTypeId,
  std::unordered_set<Entity>> &_changes) const
{
  // Get all changes
  for (const auto &[componentType, entities] :
    this->dataPtr->periodicChangedComponents)
  {
    _changes[componentType].insert(
      entities.begin(), entities.end());
  }

  // Get all removed components
  for (const auto &[entity, components] :
    this->dataPtr->componentsMarkedAsRemoved)
  {
    for (const auto &comp : components)
    {
      _changes[comp].erase(entity);
    }
  }

  // Get all removed entities
  for (const auto &entity : this->dataPtr->toRemoveEntities) {
    for (
      auto components = _changes.begin();
      components != _changes.end(); components++) {
      // Its ok to leave component entries empty, the serialization
      // code will simply ignore it. In any case the number of components
      // is limited, so this cache will never grow too large.
      components->second.erase(entity);
    }
  }
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
bool EntityComponentManager::CreateComponentImplementation(
    const Entity _entity, const ComponentTypeId _componentTypeId,
    const components::BaseComponent *_data)
{
  // make sure the entity exists
  if (!this->HasEntity(_entity))
  {
    gzerr << "Trying to create a component of type [" << _componentTypeId
      << "] attached to entity [" << _entity << "], but this entity does not "
      << "exist. This create component request will be ignored." << std::endl;
    return false;
  }

  // if this is the first time this component type is being created, make sure
  // the component type to be created is valid
  if (!this->HasComponentType(_componentTypeId) &&
      !components::Factory::Instance()->HasType(_componentTypeId))
  {
    gzerr << "Failed to create component of type [" << _componentTypeId
           << "] for entity [" << _entity
           << "]. Type has not been properly registered." << std::endl;
    return false;
  }

  // assume the component data needs to be updated externally unless this
  // component is a brand new creation/addition
  bool updateData = true;

  this->dataPtr->AddModifiedComponent(_entity);
  this->dataPtr->oneTimeChangedComponents[_componentTypeId].insert(_entity);

  // make sure the entity exists
  auto typeMapIter = this->dataPtr->componentTypeIndex.find(_entity);
  if (typeMapIter == this->dataPtr->componentTypeIndex.end())
  {
    gzerr << "Attempt to create a component of type [" << _componentTypeId
      << "] attached to entity [" << _entity
      << "] failed: entity not in componentTypeIndex." << std::endl;
    return false;
  }

  auto entityCompIter = this->dataPtr->componentStorage.find(_entity);
  if (entityCompIter == this->dataPtr->componentStorage.end())
  {
    gzerr << "Attempt to create a component of type [" << _componentTypeId
      << "] attached to entity [" << _entity
      << "] failed: entity not in storage." << std::endl;
    return false;
  }

  // Instantiate the new component.
  auto newComp = components::Factory::Instance()->New(_componentTypeId, _data);

  const auto compIdxIter = typeMapIter->second.find(_componentTypeId);
  // If entity has never had a component of this type
  if (compIdxIter == typeMapIter->second.end())
  {
    const auto vectorIdx = entityCompIter->second.size();
    entityCompIter->second.push_back(std::move(newComp));
    this->dataPtr->componentTypeIndex[_entity][_componentTypeId] = vectorIdx;
    this->dataPtr->componentTypeIndexDirty = true;

    updateData = false;
    for (auto &viewPair : this->dataPtr->views)
    {
      auto &view = viewPair.second.first;
      if (this->EntityMatches(_entity, view->ComponentTypes()))
        view->MarkEntityToAdd(_entity, this->IsNewEntity(_entity));
    }
  }
  else
  {
    // if the pre-existing component is marked as removed, this means that the
    // component was added to the entity previously, but later removed. In this
    // case, a re-addition of the component is occuring. If the pre-existing
    // component is not marked as removed, this means that the component was
    // added to the entity previously and never removed. In this case, we are
    // simply modifying the data of the pre-existing component (the modification
    // of the data is done externally in a templated ECM method call, because we
    // need the derived component class in order to update the derived component
    // data)
    auto existingCompPtr = entityCompIter->second.at(compIdxIter->second).get();
    if (!existingCompPtr)
    {
      gzerr << "Internal error: entity [" << _entity << "] has a component of "
        << "type [" << _componentTypeId << "] in the storage, but the instance "
        << "of this component is nullptr. This should never happen!"
        << std::endl;
      return false;
    }
    else if (this->dataPtr->ComponentMarkedAsRemoved(_entity, _componentTypeId))
    {
      this->dataPtr->componentsMarkedAsRemoved[_entity].erase(_componentTypeId);

      for (auto &viewPair : this->dataPtr->views)
      {
        viewPair.second.first->NotifyComponentAddition(_entity,
            this->IsNewEntity(_entity), _componentTypeId);
      }
    }
  }

  this->dataPtr->createdCompTypes.insert(_componentTypeId);

  // If the component is a components::ParentEntity, then make sure to
  // update the entities graph.
  if (_componentTypeId == components::ParentEntity::typeId)
  {
    auto parentComp = this->Component<components::ParentEntity>(_entity);
    this->SetParentEntity(_entity, parentComp->Data());
  }

  return updateData;
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityMatches(Entity _entity,
    const std::set<ComponentTypeId> &_types) const
{
  auto iter = this->dataPtr->componentTypeIndex.find(_entity);
  if (iter == this->dataPtr->componentTypeIndex.end())
    return false;

  // quick check: the entity cannot match _types if _types is larger than the
  // number of component types the entity has
  if (_types.size() > iter->second.size())
    return false;

  // \todo(nkoenig) The performance of this could be improved.
  // It might be possible to create bitmask for component sets.
  // Fixing this might not be high priority, unless we expect frequent
  // creation of entities and/or queries.
  for (const ComponentTypeId &type : _types)
  {
    auto typeIter = iter->second.find(type);
    if (typeIter == iter->second.end() ||
        this->dataPtr->ComponentMarkedAsRemoved(_entity, type))
      return false;
  }

  return true;
}

/////////////////////////////////////////////////
const components::BaseComponent
    *EntityComponentManager::ComponentImplementation(
    const Entity _entity, const ComponentTypeId _type) const
{
  GZ_PROFILE("EntityComponentManager::ComponentImplementation");

  // make sure the entity exists
  const auto typeMapIter = this->dataPtr->componentTypeIndex.find(_entity);
  if (typeMapIter == this->dataPtr->componentTypeIndex.end())
    return nullptr;

  // make sure the component type exists for the entity
  const auto compIdxIter = typeMapIter->second.find(_type);
  if (compIdxIter == typeMapIter->second.end())
    return nullptr;

  // get the pointer to the component
  const auto compVecIter = this->dataPtr->componentStorage.find(_entity);
  if (compVecIter == this->dataPtr->componentStorage.end())
  {
    gzerr << "Internal error: Entity [" << _entity
      << "] is missing in storage, but is in "
      << "componentTypeIndex. This should never happen!" << std::endl;
    return nullptr;
  }

  auto compPtr = compVecIter->second.at(compIdxIter->second).get();
  if (nullptr == compPtr)
  {
    gzerr << "Internal error: entity [" << _entity << "] has a component of "
      << "type [" << _type << "] in the storage, but the instance "
      << "of this component is nullptr. This should never happen!"
      << std::endl;
    return nullptr;
  }

  // Return component if not marked as removed.
  if (!this->dataPtr->ComponentMarkedAsRemoved(_entity, _type))
    return compPtr;

  return nullptr;
}

/////////////////////////////////////////////////
components::BaseComponent *EntityComponentManager::ComponentImplementation(
    const Entity _entity, const ComponentTypeId _type)
{
  // Call the const version of the function
  return const_cast<components::BaseComponent *>(
      static_cast<const EntityComponentManager &>(
      *this).ComponentImplementation(_entity, _type));
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasComponentType(
    const ComponentTypeId _typeId) const
{
  return this->dataPtr->createdCompTypes.find(_typeId) !=
    this->dataPtr->createdCompTypes.end();
}

//////////////////////////////////////////////////
const EntityGraph &EntityComponentManager::Entities() const
{
  return this->dataPtr->entities;
}

//////////////////////////////////////////////////
std::pair<detail::BaseView *, std::mutex *> EntityComponentManager::FindView(
    const std::vector<ComponentTypeId> &_types) const
{
  std::lock_guard<std::mutex> lockViews(this->dataPtr->viewsMutex);
  std::pair<detail::BaseView *, std::mutex *> viewMutexPair(nullptr, nullptr);
  auto iter = this->dataPtr->views.find(_types);
  if (iter != this->dataPtr->views.end())
  {
    viewMutexPair.first = iter->second.first.get();
    viewMutexPair.second = iter->second.second.get();
  }
  return viewMutexPair;
}

//////////////////////////////////////////////////
detail::BaseView *EntityComponentManager::AddView(
    const detail::ComponentTypeKey &_types,
    std::unique_ptr<detail::BaseView> _view) const
{
  // If the view already exists, then the map will return the iterator to
  // the location that prevented the insertion.
  std::lock_guard<std::mutex> lockViews(this->dataPtr->viewsMutex);
  auto iter = this->dataPtr->views.insert(std::make_pair(_types,
        std::make_pair(std::move(_view),
          std::make_unique<std::mutex>()))).first;
  return iter->second.first.get();
}

//////////////////////////////////////////////////
void EntityComponentManager::RebuildViews()
{
  GZ_PROFILE("EntityComponentManager::RebuildViews");
  for (auto &viewPair : this->dataPtr->views)
  {
    auto &view = viewPair.second.first;
    view->Reset();

    // Add all the entities that match the component types to the
    // view.
    for (const auto &vertex : this->dataPtr->entities.Vertices())
    {
      Entity entity = vertex.first;
      if (this->EntityMatches(entity, view->ComponentTypes()))
      {
        view->MarkEntityToAdd(entity, this->IsNewEntity(entity));

        // If there is a request to delete this entity, update the view as
        // well
        if (this->IsMarkedForRemoval(entity))
          view->MarkEntityToRemove(entity);
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
  auto entRemovedCompsIter = this->removedComponents.find(_entity);
  if (entRemovedCompsIter == this->removedComponents.end())
    return;
  for (const auto &compType : entRemovedCompsIter->second)
  {
    if (!_types.empty() && _types.find(compType) == _types.end())
    {
      continue;
    }

    auto compMsg = _entityMsg->add_components();

    // Empty data is needed for the component to be processed afterwards
    compMsg->set_component(" ");
    compMsg->set_type(compType);
    compMsg->set_remove(true);
  }
}

//////////////////////////////////////////////////
void EntityComponentManagerPrivate::SetRemovedComponentsMsgs(Entity &_entity,
    msgs::SerializedStateMap &_msg,
    const std::unordered_set<ComponentTypeId> &_types)
{
  std::lock_guard<std::mutex> lock(this->removedComponentsMutex);
  auto entRemovedCompsIter = this->removedComponents.find(_entity);
  if (entRemovedCompsIter == this->removedComponents.end())
    return;
  uint64_t nEntityKeys = entRemovedCompsIter->second.size();
  if (nEntityKeys == 0)
    return;

  // The message need not necessarily contain the entity initially. For
  // instance, when AddEntityToMessage() calls this function, the entity may
  // have some removed components but none that changed,
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

  for (const auto &compType : entRemovedCompsIter->second)
  {
    if (!_types.empty() && _types.find(compType) == _types.end())
    {
      continue;
    }

    msgs::SerializedComponent compMsg;

    // Empty data is needed for the component to be processed afterwards
    compMsg.set_component(" ");
    compMsg.set_type(compType);
    compMsg.set_remove(true);

    (*(entIter->second.mutable_components()))[
      static_cast<int64_t>(compType)] = compMsg;
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::AddEntityToMessage(msgs::SerializedState &_msg,
    Entity _entity, const std::unordered_set<ComponentTypeId> &_types) const
{
  auto entityMsg = _msg.add_entities();
  entityMsg->set_id(_entity);
  auto iter = this->dataPtr->componentTypeIndex.find(_entity);
  if (iter == this->dataPtr->componentTypeIndex.end())
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
    for (auto &type : this->dataPtr->componentTypeIndex[_entity])
    {
      if (!this->dataPtr->ComponentMarkedAsRemoved(_entity, type.first))
        types.insert(type.first);
    }
  }

  for (const ComponentTypeId type : types)
  {
    // If the entity does not have the component, continue
    auto typeIter = iter->second.find(type);
    if (typeIter == iter->second.end())
      continue;

    // The component instance is nullptr if the component was removed
    auto compBase = this->ComponentImplementation(_entity, type);
    if (nullptr == compBase)
      continue;

    auto compMsg = entityMsg->add_components();
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
  auto iter = this->dataPtr->componentTypeIndex.find(_entity);
  if (iter == this->dataPtr->componentTypeIndex.end())
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
    for (auto &type : this->dataPtr->componentTypeIndex[_entity])
    {
      if (!this->dataPtr->ComponentMarkedAsRemoved(_entity, type.first))
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

    // If not sending full state, skip unchanged components
    if (!_full)
    {
      bool noChange = true;

      // see if the entity has a component of this particular type marked as a
      // one time change
      auto oneTimeIter = this->dataPtr->oneTimeChangedComponents.find(type);
      if (oneTimeIter != this->dataPtr->oneTimeChangedComponents.end() &&
          oneTimeIter->second.find(_entity) != oneTimeIter->second.end())
        noChange = false;

      if (noChange)
      {
        // see if the entity has a component of this particular type marked as a
        // periodic change
        auto periodicIter = this->dataPtr->periodicChangedComponents.find(type);
        if (periodicIter != this->dataPtr->periodicChangedComponents.end() &&
            periodicIter->second.find(_entity) != periodicIter->second.end())
          noChange = false;
      }

      if (noChange)
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

    auto compIter = entIter->second.mutable_components()->find(type);
    // Find the component in the message, and add the component to the
    // message if it's not present.
    if (compIter == entIter->second.mutable_components()->end())
    {
      msgs::SerializedComponent cmp;
      cmp.set_type(compBase->TypeId());
      (*(entIter->second.mutable_components()))[
        static_cast<int64_t>(type)] = cmp;
      compIter = entIter->second.mutable_components()->find(type);
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
msgs::SerializedState EntityComponentManager::ChangedState() const
{
  msgs::SerializedState stateMsg;

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
    msgs::SerializedStateMap &_state) const
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
  // threads and each thread's work load
  if (!this->componentTypeIndexDirty)
    return;

  this->componentTypeIndexDirty = false;
  this->componentTypeIndexIterators.clear();
  auto startIt = this->componentTypeIndex.begin();
  int numEntities = this->componentTypeIndex.size();

  // Set the number of threads to spawn to the min of the calculated thread
  // count or max threads that the hardware supports
  int maxThreads = std::thread::hardware_concurrency();
  uint64_t numThreads = std::min(numEntities, maxThreads);

  int entitiesPerThread = static_cast<int>(std::ceil(
    static_cast<double>(numEntities) / numThreads));

  gzdbg << "Updated state thread iterators: " << numThreads
         << " threads processing around " << entitiesPerThread
         << " entities each." << std::endl;

  // Push back the starting iterator
  this->componentTypeIndexIterators.push_back(startIt);
  for (uint64_t i = 0; i < numThreads; ++i)
  {
    // If we have added all of the entities to the iterator vector, we are
    // done so push back the end iterator
    numEntities -= entitiesPerThread;
    if (numEntities <= 0)
    {
      this->componentTypeIndexIterators.push_back(
          this->componentTypeIndex.end());
      break;
    }

    // Get the iterator to the next starting group of entities
    auto nextIt = std::next(startIt, entitiesPerThread);
    this->componentTypeIndexIterators.push_back(nextIt);
    startIt = nextIt;
  }
}

//////////////////////////////////////////////////
msgs::SerializedState EntityComponentManager::State(
    const std::unordered_set<Entity> &_entities,
    const std::unordered_set<ComponentTypeId> &_types) const
{
  msgs::SerializedState stateMsg;
  for (const auto &it : this->dataPtr->componentTypeIndex)
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
  uint64_t numThreads = this->dataPtr->componentTypeIndexIterators.size() - 1;
  for (uint64_t i = 0; i < numThreads; i++)
  {
    workers.push_back(std::thread(functor,
        this->dataPtr->componentTypeIndexIterators[i],
        this->dataPtr->componentTypeIndexIterators[i+1]));
  }

  // Wait for each thread to finish processing its components
  std::for_each(workers.begin(), workers.end(), [](std::thread &_t)
  {
    _t.join();
  });
}

//////////////////////////////////////////////////
void EntityComponentManager::PeriodicStateFromCache(
  msgs::SerializedStateMap &_state,
  const std::unordered_map<ComponentTypeId,
    std::unordered_set<Entity>> &_cache) const
{
  for (auto &[typeId, entities] : _cache) {
    // Serialize components that have changed
    for (auto &entity : entities) {
      // Add entity to message if it does not exist
      auto entIter = _state.mutable_entities()->find(entity);
      if (entIter == _state.mutable_entities()->end())
      {
        msgs::SerializedEntityMap ent;
        ent.set_id(entity);
        (*_state.mutable_entities())[static_cast<uint64_t>(entity)] = ent;
        entIter = _state.mutable_entities()->find(entity);
      }

      // Find the component in the message
      auto compIter = entIter->second.mutable_components()->find(typeId);
      if (compIter != entIter->second.mutable_components()->end())
      {
        // If the component is present we don't need to update it.
        continue;
      }

      auto compIdx = this->dataPtr->componentTypeIndex[entity][typeId];
      auto &comp = this->dataPtr->componentStorage[entity][compIdx];

      // Add the component to the message
      msgs::SerializedComponent cmp;
      cmp.set_type(comp->TypeId());
      std::ostringstream ostr;
      comp->Serialize(ostr);
      cmp.set_component(ostr.str());
      (*(entIter->second.mutable_components()))[
      static_cast<int64_t>(typeId)] = cmp;
    }
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::SetState(
    const msgs::SerializedState &_stateMsg)
{
  GZ_PROFILE("EntityComponentManager::SetState Non-map");
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
          gzwarn << "Component type [" << type << "] has not been "
                  << "registered in this process, so it can't be deserialized."
                  << std::endl;
        }
        continue;
      }

      // Remove component
      if (compMsg.remove())
      {
        this->RemoveComponent(entity, type);
        continue;
      }

      // Get Component
      auto comp = this->ComponentImplementation(entity, type);

      // Create if new
      if (nullptr == comp)
      {
        std::istringstream istr(compMsg.component());

        auto newComp = components::Factory::Instance()->New(type);
        if (nullptr == newComp)
        {
          gzerr << "Failed to create component type ["
            << compMsg.type() << "]" << std::endl;
          continue;
        }
        newComp->Deserialize(istr);

        auto updateData =
          this->CreateComponentImplementation(entity, type, newComp.get());
        if (updateData)
        {
          // Set comp so we deserialize the data below again
          comp = this->ComponentImplementation(entity, type);
        }
      }

      // Update component value
      if (comp)
      {
        std::istringstream istr(compMsg.component());
        comp->Deserialize(istr);
        this->dataPtr->AddModifiedComponent(entity);
      }
    }
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::SetState(
    const msgs::SerializedStateMap &_stateMsg)
{
  GZ_PROFILE("EntityComponentManager::SetState Map");
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

      uint64_t type = compMsg.type();

      // Components which haven't been registered in this process, such as 3rd
      // party components streamed to other secondaries and the GUI.
      if (!components::Factory::Instance()->HasType(type))
      {
        static std::unordered_set<unsigned int> printedComps;
        if (printedComps.find(type) == printedComps.end())
        {
          printedComps.insert(type);
          gzwarn << "Component type [" << type << "] has not been "
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
        std::istringstream istr(compMsg.component());

        // Create component
        auto newComp = components::Factory::Instance()->New(compMsg.type());
        if (nullptr == newComp)
        {
          gzerr << "Failed to create component of type [" << compMsg.type()
            << "]" << std::endl;
          continue;
        }
        newComp->Deserialize(istr);

        auto updateData = this->CreateComponentImplementation(
          entity, newComp->TypeId(), newComp.get());
        if (updateData)
        {
          // Set comp so we deserialize the data below again
          comp = this->ComponentImplementation(entity, compIter.first);
        }
      }

      // Update component value
      if (comp)
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
    sim::ComponentState _c)
{
  // make sure _entity exists
  auto ecIter = this->dataPtr->componentTypeIndex.find(_entity);
  if (ecIter == this->dataPtr->componentTypeIndex.end())
    return;

  // make sure the entity has a component of type _type
  if (ecIter->second.find(_type) == ecIter->second.end() ||
      this->dataPtr->ComponentMarkedAsRemoved(_entity, _type))
    return;

  if (_c == ComponentState::PeriodicChange)
  {
    this->dataPtr->periodicChangedComponents[_type].insert(_entity);
    auto oneTimeIter = this->dataPtr->oneTimeChangedComponents.find(_type);
    if (oneTimeIter != this->dataPtr->oneTimeChangedComponents.end())
      oneTimeIter->second.erase(_entity);
  }
  else if (_c == ComponentState::OneTimeChange)
  {
    auto periodicIter = this->dataPtr->periodicChangedComponents.find(_type);
    if (periodicIter != this->dataPtr->periodicChangedComponents.end())
      periodicIter->second.erase(_entity);
    this->dataPtr->oneTimeChangedComponents[_type].insert(_entity);
  }
  else
  {
    auto periodicIter = this->dataPtr->periodicChangedComponents.find(_type);
    if (periodicIter != this->dataPtr->periodicChangedComponents.end())
      periodicIter->second.erase(_entity);
    auto oneTimeIter = this->dataPtr->oneTimeChangedComponents.find(_type);
    if (oneTimeIter != this->dataPtr->oneTimeChangedComponents.end())
      oneTimeIter->second.erase(_entity);

    // the component state is flagged as no change, so don't mark the
    // corresponding entity as one with a modified component
    return;
  }

  this->dataPtr->AddModifiedComponent(_entity);
}

/////////////////////////////////////////////////
std::unordered_set<ComponentTypeId> EntityComponentManager::ComponentTypes(
    const Entity _entity) const
{
  auto it = this->dataPtr->componentTypeIndex.find(_entity);
  if (it == this->dataPtr->componentTypeIndex.end())
    return {};

  std::unordered_set<ComponentTypeId> result;
  for (const auto &type : it->second)
  {
    if (!this->dataPtr->ComponentMarkedAsRemoved(_entity, type.first))
      result.insert(type.first);
  }

  return result;
}

/////////////////////////////////////////////////
void EntityComponentManager::SetEntityCreateOffset(uint64_t _offset)
{
  if (_offset < this->dataPtr->entityCount)
  {
    gzwarn << "Setting an entity offset of [" << _offset << "] is less than "
     << "the current entity count of [" << this->dataPtr->entityCount << "]. "
     << "Incorrect behavior should be expected.\n";
  }

  this->dataPtr->entityCount = _offset;
}

/////////////////////////////////////////////////
void EntityComponentManager::LockAddingEntitiesToViews(bool _lock)
{
  this->dataPtr->lockAddEntitiesToViews = _lock;
}

/////////////////////////////////////////////////
bool EntityComponentManager::LockAddingEntitiesToViews() const
{
  return this->dataPtr->lockAddEntitiesToViews;
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

/////////////////////////////////////////////////
bool EntityComponentManagerPrivate::ComponentMarkedAsRemoved(
    const Entity _entity, const ComponentTypeId _typeId) const
{
  auto iter = this->componentsMarkedAsRemoved.find(_entity);
  if (iter != this->componentsMarkedAsRemoved.end())
    return iter->second.find(_typeId) != iter->second.end();

  return false;
}

/////////////////////////////////////////////////
template<typename ComponentTypeT>
bool EntityComponentManagerPrivate::ClonedJointLinkName(Entity _joint,
    Entity _originalLink, EntityComponentManager *_ecm)
{
  if (ComponentTypeT::typeId != components::ParentLinkName::typeId &&
      ComponentTypeT::typeId != components::ChildLinkName::typeId)
  {
    gzerr << "Template type is invalid. Must be either "
           << "components::ParentLinkName or components::ChildLinkName\n";
    return false;
  }

  Entity clonedLink = kNullEntity;


  std::string name;
  // Handle the case where the link could have been the world.
  if (_ecm->Component<components::World>(_originalLink) != nullptr)
  {
    // Use the special identifier "world".
    name = "world";
  }
  else
  {
    auto iter = this->originalToClonedLink.find(_originalLink);
    if (iter == this->originalToClonedLink.end())
    {
      gzerr << "Error: attempted to clone links, but link ["
        << _originalLink << "] was never cloned.\n";
      return false;
    }
    clonedLink = iter->second;

    auto nameComp = _ecm->Component<components::Name>(clonedLink);
    if (!nameComp)
    {
      gzerr << "Link [" << _originalLink
        << "] was cloned, but its clone has no name.\n";
      return false;
    }
    name = nameComp->Data();
  }

  _ecm->SetComponentData<ComponentTypeT>(_joint, name);
  return true;
}

/////////////////////////////////////////////////
void EntityComponentManager::PinEntity(const Entity _entity, bool _recursive)
{
  if (_recursive)
  {
    this->dataPtr->InsertEntityRecursive(_entity,
        this->dataPtr->pinnedEntities);
  }
  else
  {
    this->dataPtr->pinnedEntities.insert(_entity);
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::UnpinEntity(const Entity _entity, bool _recursive)
{
  if (_recursive)
  {
    this->dataPtr->EraseEntityRecursive(_entity,
        this->dataPtr->pinnedEntities);
  }
  else
  {
    this->dataPtr->pinnedEntities.erase(_entity);
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::UnpinAllEntities()
{
  this->dataPtr->pinnedEntities.clear();
}

/////////////////////////////////////////////////
void EntityComponentManager::CopyFrom(const EntityComponentManager &_fromEcm)
{
  this->dataPtr->CopyFrom(*_fromEcm.dataPtr);
}

/////////////////////////////////////////////////
EntityComponentManagerDiff EntityComponentManager::ComputeEntityDiff(
    const EntityComponentManager &_other) const
{
  EntityComponentManagerDiff diff;
  for (const auto &item : _other.dataPtr->entities.Vertices())
  {
    const auto &v = item.second.get();
    if (!this->dataPtr->entities.VertexFromId(v.Id()).Valid())
    {
      // In `_other` but not in `this`, so insert the entity as an "added"
      // entity.
      diff.InsertAddedEntity(v.Data());
    }
  }

  for (const auto &item : this->dataPtr->entities.Vertices())
  {
    const auto &v = item.second.get();
    if (!_other.dataPtr->entities.VertexFromId(v.Id()).Valid())
    {
      // In `this` but not in `other`, so insert the entity as a "removed"
      // entity.
      diff.InsertRemovedEntity(v.Data());
    }
  }
  return diff;
}

/////////////////////////////////////////////////
void EntityComponentManager::ApplyEntityDiff(
    const EntityComponentManager &_other,
    const EntityComponentManagerDiff &_diff)
{
  auto copyComponents = [&](Entity _entity)
  {
    for (const auto compTypeId : _other.ComponentTypes(_entity))
    {
      const components::BaseComponent *data =
          _other.ComponentImplementation(_entity, compTypeId);
      this->CreateComponentImplementation(_entity, compTypeId,
                                          data->Clone().get());
    }
  };

  for(auto entity : _diff.AddedEntities())
  {
    if (!this->HasEntity(entity))
    {
      this->dataPtr->CreateEntityImplementation(entity);
      if (entity >= this->dataPtr->entityCount)
      {
        this->dataPtr->entityCount = entity;
      }
      copyComponents(entity);
      this->SetParentEntity(entity, _other.ParentEntity(entity));
    }
  }

  for (const auto &entity : _diff.RemovedEntities())
  {
    // if the entity is not in this ECM, add it before requesting for its
    // removal.
    if (!this->HasEntity(entity))
    {
      this->dataPtr->CreateEntityImplementation(entity);
      // We want to set this entity as "removed", but
      // CreateEntityImplementation sets it as "newlyCreated",
      // so remove it from that list.
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
        this->dataPtr->newlyCreatedEntities.erase(entity);
      }
      // Copy components so that EachRemoved match correctly
      if (entity >= this->dataPtr->entityCount)
      {
        this->dataPtr->entityCount = entity;
      }
      copyComponents(entity);
      this->SetParentEntity(entity, _other.ParentEntity(entity));
    }

    this->RequestRemoveEntity(entity, false);
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::ResetTo(const EntityComponentManager &_other)
{
  auto ecmDiff = this->ComputeEntityDiff(_other);
  EntityComponentManager tmpCopy;
  tmpCopy.CopyFrom(_other);
  tmpCopy.ApplyEntityDiff(*this, ecmDiff);
  this->CopyFrom(tmpCopy);
}

/////////////////////////////////////////////////
std::optional<Entity> EntityComponentManager::EntityByName(
    const std::string &_name) const
{
  std::optional<Entity> entity;
   Entity entByName = EntityByComponents(components::Name(_name));
  if (entByName != kNullEntity)
    entity = entByName;

  return entity;
}
