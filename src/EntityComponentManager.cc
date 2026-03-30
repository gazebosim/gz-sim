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
#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Recreate.hh"
#include "gz/sim/components/World.hh"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#include "gz/sim/entt/entity/handle.hpp"
#pragma GCC diagnostic pop

using namespace gz;
using namespace sim;

struct PinnedEntity { };

class gz::sim::EntityComponentManagerPrivate
{
  /// \brief Implementation of the CreateEntity function, which takes a specific
  /// entity as input.
  /// \param[in] _entity Entity to be created.
  /// \return Created entity, which should match the input.
  public: Entity CreateEntityImplementation(entt::basic_registry<Entity>& _registry, Entity _entity);

  /// \brief Recursively insert an entity and all its descendants into a given
  /// set.
  /// \param[in] _entity Entity to be inserted.
  /// \param[in, out] _set Set to be filled.
  public: void InsertEntityRecursive(const entt::basic_registry<Entity>& _registry, Entity _entity,
      std::unordered_set<Entity> &_set);

  /// \brief Allots the work for multiple threads prior to running
  /// `AddEntityToMessage`.
  public: void CalculateStateThreadLoad(const entt::basic_registry<Entity>& _registry);

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
  public: void AddModifiedComponent(entt::basic_registry<Entity>& _registry, const Entity &_entity);

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
  /// This is a temporary workaround until we find a way to clone entities and
  /// components that don't require special treatment for particular component
  /// types.
  public: template<typename ComponentTypeT>
          bool ClonedJointLinkName(Entity _joint, Entity _originalLink,
              EntityComponentManager *_ecm);
          /*

  /// \brief All component types that have ever been created.
  public: std::unordered_set<ComponentTypeId> createdCompTypes;

          */
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

  /// \brief A mutex to protect newly created entities.
  public: std::mutex entityCreatedMutex;

  /// \brief A mutex to protect entity remove.
  public: std::mutex entityRemoveMutex;

  /// \brief A mutex to protect removed components
  public: mutable std::mutex removedComponentsMutex;

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

  /// \brief Groups that are pending to be created.
  public: std::vector<std::unique_ptr<detail::GroupQueuer>> pendingGroups;

  /// \brief Set of group component types that are already enqueued.
  public: std::set<std::vector<ComponentTypeId>> enqueuedGroups;

  /// \brief Mutex to protect enqueuedGroups and pendingGroups.
  public: mutable std::mutex groupMutex;
};

//////////////////////////////////////////////////
void EntityComponentManager::CreatePendingGroups()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->groupMutex);
  for (auto &group : this->dataPtr->pendingGroups)
  {
    group->CreateGroup(this->registry);
  }
  this->dataPtr->pendingGroups.clear();
  this->dataPtr->enqueuedGroups.clear();
}

//////////////////////////////////////////////////
void EntityComponentManager::EnqueueGroup(
    const std::vector<ComponentTypeId> &_types,
    std::unique_ptr<detail::GroupQueuer> _queuer) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->groupMutex);
  if (this->dataPtr->enqueuedGroups.insert(_types).second)
  {
    this->dataPtr->pendingGroups.push_back(std::move(_queuer));
  }
}

//////////////////////////////////////////////////
EntityComponentManager::EntityComponentManager()
  : dataPtr(new EntityComponentManagerPrivate)
{
  components::Factory::Instance()->RegisterAllToEntt(this->registry);
}

//////////////////////////////////////////////////
EntityComponentManager::~EntityComponentManager() = default;

//////////////////////////////////////////////////
void EntityComponentManagerPrivate::CopyFrom(
    const EntityComponentManagerPrivate &_from)
{
  this->periodicChangedComponents = _from.periodicChangedComponents;
  this->oneTimeChangedComponents = _from.oneTimeChangedComponents;
  this->entityCount = _from.entityCount;
  this->removedComponents = _from.removedComponents;
  this->componentsMarkedAsRemoved = _from.componentsMarkedAsRemoved;

  // Not copying maps related to cloning since they are transient variables
  // that are used as return values of some member functions.
}

//////////////////////////////////////////////////
size_t EntityComponentManager::EntityCount() const
{
  return this->registry.storage<Entity>()->free_list();
}

/////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntity()
{
  this->dataPtr->entityCount++;
  return this->dataPtr->CreateEntityImplementation(this->registry, this->dataPtr->entityCount);
}

/////////////////////////////////////////////////
Entity EntityComponentManagerPrivate::CreateEntityImplementation(entt::basic_registry<Entity>& _registry, Entity _entity)
{
  GZ_PROFILE("EntityComponentManager::CreateEntityImplementation");
  const auto e = _registry.create(_entity);
  _registry.emplace<NewEntity>(e);
  _registry.emplace<Children>(e);
  return e;
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
  this->registry.clear<NewEntity>();
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
void EntityComponentManagerPrivate::InsertEntityRecursive(const entt::basic_registry<Entity>& _registry, Entity _entity,
    std::unordered_set<Entity> &_set)
{
  _set.insert(_entity);
  const auto* children = _registry.try_get<Children>(_entity);
  if (!children)
    return;
  for (const auto& child : children->data)
  {
    this->InsertEntityRecursive(_registry, child, _set);
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestRemoveEntity(Entity _entity,
    bool _recursive)
{
  if (!this->HasEntity(_entity))
    return;
  // Store the to-be-removed entities in a temporary set so we can call
  // UpdateViews on each of them
  std::unordered_set<Entity> tmpToRemoveEntities;
  if (!_recursive)
  {
    tmpToRemoveEntities.insert(_entity);
  }
  else
  {
    this->dataPtr->InsertEntityRecursive(this->registry, _entity, tmpToRemoveEntities);

    // remove detachable joint entities that are connected to
    // any of the entities to be removed
    std::unordered_set<Entity> detachableJoints;
    this->Each<components::DetachableJoint>(
        [&](const Entity &_jointEntity,
            const components::DetachableJoint *_jointInfo) -> bool
        {
          Entity parentLinkEntity = _jointInfo->Data().parentLink;
          Entity childLinkEntity = _jointInfo->Data().childLink;
          if (tmpToRemoveEntities.find(parentLinkEntity) !=
              tmpToRemoveEntities.end() ||
              tmpToRemoveEntities.find(childLinkEntity) !=
              tmpToRemoveEntities.end())
            detachableJoints.insert(_jointEntity);
          return true;
        });
    tmpToRemoveEntities.insert(detachableJoints.begin(),
                               detachableJoints.end());
  }

  // Remove entities from tmpToRemoveEntities that are marked as
  // unremovable.
  std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
  const auto& storage = this->registry.storage<PinnedEntity>();
  for (const auto& e : tmpToRemoveEntities)
  {
    if (!storage.contains(e))
    {
      // Not pinned, erase
      this->registry.emplace<RemoveEntity>(e);
    }
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestRemoveEntities()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
  this->registry.view<Entity>(entt::exclude<PinnedEntity>).each([&](const Entity _e) {
    this->registry.emplace<RemoveEntity>(_e);
  });
}

/////////////////////////////////////////////////
void EntityComponentManager::ProcessRemoveEntityRequests()
{
  GZ_PROFILE("EntityComponentManager::ProcessRemoveEntityRequests");
  std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
  auto view = this->registry.view<RemoveEntity>();
  const auto& pinnedStorage = this->registry.storage<PinnedEntity>();
  for (auto entity : view)
  {
    if (pinnedStorage.contains(entity))
    {
      continue;
    }
    const auto* parent = this->registry.try_get<components::ParentEntity>(entity);
    if (parent)
    {
      auto* parentChildren = this->registry.try_get<Children>(parent->Data());
      if (parentChildren)
      {
        parentChildren->data.erase(entity);
      }
    }
    this->registry.destroy(entity);
    this->dataPtr->componentsMarkedAsRemoved.erase(entity);
  }
}

/////////////////////////////////////////////////
bool EntityComponentManager::RemoveComponent(
    const Entity _entity, const ComponentTypeId &_typeId)
{
  GZ_PROFILE("EntityComponentManager::RemoveComponent");
  // Make sure the entity exists and has the component.
  if (!this->HasEntity(_entity))
    return false;

  auto* storage = this->registry.storage(_typeId);

  if (storage && storage->contains(_entity)) {
      storage->remove(_entity);
      this->PostRemoveComponent(_entity, _typeId);
      return true;
  }
  return false;
}

/////////////////////////////////////////////////
void EntityComponentManager::PostRemoveComponent(const Entity _entity, const ComponentTypeId &_typeId)
{
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

  this->dataPtr->AddModifiedComponent(this->registry, _entity);

  // Add component to map of removed components
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->removedComponentsMutex);
    this->dataPtr->removedComponents[_entity].insert(_typeId);
  }

  this->MarkComponentAsRemoved(_entity, _typeId, true);
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponentType(const Entity _entity,
    const ComponentTypeId &_typeId) const
{
  if (!this->HasEntity(_entity))
    return false;
  const auto* storage = this->registry.storage(_typeId);
  if (storage == nullptr)
    return false;
  return storage->contains(_entity);
}

/////////////////////////////////////////////////
bool EntityComponentManager::IsMarkedForRemoval(const Entity _entity) const
{
  if (!this->HasEntity(_entity))
    return false;
  std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
  return this->registry.any_of<RemoveEntity>(_entity);
}

/////////////////////////////////////////////////
ComponentState EntityComponentManager::ComponentState(const Entity _entity,
    const ComponentTypeId _typeId) const
{
  auto result = ComponentState::NoChange;

  if (!this->HasEntity(_entity))
    return result;

  if (this->dataPtr->ComponentMarkedAsRemoved(_entity, _typeId))
    return result;

  auto oneTimeIter = this->dataPtr->oneTimeChangedComponents.find(_typeId);
  if (oneTimeIter != this->dataPtr->oneTimeChangedComponents.end() &&
      oneTimeIter->second.find(_entity) != oneTimeIter->second.end())
  {
    result = ComponentState::OneTimeChange;
  }
  else
  {
    auto periodicIter =
      this->dataPtr->periodicChangedComponents.find(_typeId);
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
  return this->registry.view<NewEntity>().size() > 0;
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasEntitiesMarkedForRemoval() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityRemoveMutex);
  return this->registry.view<RemoveEntity>().size() > 0;
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
  this->registry.view<const RemoveEntity>().each([this, &_changes](const Entity e, const RemoveEntity&) {
    for (
      auto components = _changes.begin();
      components != _changes.end(); components++) {
      // Its ok to leave component entries empty, the serialization
      // code will simply ignore it. In any case the number of components
      // is limited, so this cache will never grow too large.
      components->second.erase(e);
    }
  });
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasEntity(const Entity _entity) const
{
  return this->registry.valid(_entity);
}

/////////////////////////////////////////////////
Entity EntityComponentManager::ParentEntity(const Entity _entity) const
{
  const auto* parent = this->registry.try_get<components::ParentEntity>(_entity);
  if (!parent)
    return kNullEntity;
  return parent->Data();
}

/////////////////////////////////////////////////
bool EntityComponentManager::SetParentEntity(const Entity _child,
    const Entity _parent)
{
  if (!this->HasEntity(_child))
    return false;
  if (_parent != kNullEntity && !this->HasEntity(_parent))
    return false;
  auto* currentParent = this->registry.try_get<components::ParentEntity>(_child);
  // Remove current parent(s)
  if (currentParent)
  {
    auto* children = this->registry.try_get<Children>(currentParent->Data());
    if (children)
    {
      children->data.erase(_child);
    }
  }

  // Leave parent-less
  if (_parent == kNullEntity)
  {
    this->registry.remove<components::ParentEntity>(_child);
    return true;
  }

  // Update parent entity and parent's children
  this->registry.emplace_or_replace<components::ParentEntity>(_child, _parent);
  auto& newChildren = this->registry.get<Children>(_parent);
  newChildren.data.insert(_child);
  return true;
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

  this->dataPtr->AddModifiedComponent(this->registry, _entity);
  this->dataPtr->oneTimeChangedComponents[_componentTypeId].insert(_entity);

  // Instantiate the new component.
  auto newComp = components::Factory::Instance()->New(_componentTypeId, _data);

  auto* storage = this->registry.storage(_componentTypeId);
  // If entity has never had a component of this type
  if (!storage || !storage->contains(_entity))
  {
    if (!components::Factory::Instance()->SyncComponent(this->registry, _entity, _componentTypeId, newComp.get()))
    {
      gzwarn << "Failed syncing component. This should not happen." << std::endl;
    } else {
      updateData = false;
    }
  }
  else
  {
    // if the pre-existing component is marked as removed, this means that the
    // component was added to the entity previously, but later removed. In this
    // case, a re-addition of the component is occurring. If the pre-existing
    // component is not marked as removed, this means that the component was
    // added to the entity previously and never removed. In this case, we are
    // simply modifying the data of the pre-existing component (the modification
    // of the data is done externally in a templated ECM method call, because we
    // need the derived component class in order to update the derived component
    // data)
    this->MarkComponentAsRemoved(_entity, _componentTypeId, false);
  }

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
  if (!this->HasEntity(_entity))
    return false;

  const auto entityTypes = this->ComponentTypes(_entity);
  // quick check: the entity cannot match _types if _types is larger than the
  // number of component types the entity has
  if (_types.size() > entityTypes.size())
    return false;

  // \todo(nkoenig) The performance of this could be improved.
  // It might be possible to create bitmask for component sets.
  // Fixing this might not be high priority, unless we expect frequent
  // creation of entities and/or queries.
  for (const ComponentTypeId &type : _types)
  {
    auto typeIter = entityTypes.find(type);
    if (typeIter == entityTypes.end() ||
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

  const auto* storage = this->registry.storage(_type);
  if (!storage)
    return nullptr;

  if (!storage->contains(_entity))
    return nullptr;

  // TODO(luca) Document why this is safe
  return static_cast<const components::BaseComponent*>(storage->value(_entity));
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
  const auto* storage = this->registry.storage(_typeId);
  return storage != nullptr;
}

//////////////////////////////////////////////////
const std::vector<Entity> EntityComponentManager::Entities() const
{
  std::vector<Entity> entities(this->EntityCount());
  this->registry.view<Entity>().each([&](const Entity& e) {
    entities.push_back(e);
  });
  return entities;
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
  
  if (this->registry.any_of<RemoveEntity>(_entity))
  {
    entityMsg->set_remove(true);
  }

  for (const auto& type : this->ComponentTypes(_entity)) {
    if (!_types.empty() && _types.find(type) == _types.end())
    {
      continue;
    }
    // Check for removal here
    if (this->dataPtr->ComponentMarkedAsRemoved(_entity, type))
    {
      continue;
    }

    const components::BaseComponent *compBase =
      this->ComponentImplementation(_entity, type);

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
  if (!this->HasEntity(_entity))
    return;

  // Set the default entity iterator to the end. This will allow us to know
  // if the entity has been added to the message.
  auto entIter = _msg.mutable_entities()->end();
  // Add an entity to the message and set it to be removed if the entity
  // exists in the toRemoveEntities list.
  if (this->registry.any_of<RemoveEntity>(_entity))
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
  for (const auto& type : this->ComponentTypes(_entity)) {
    if (!_types.empty() && _types.find(type) == _types.end())
    {
      continue;
    }
    // Check for removal here
    if (this->dataPtr->ComponentMarkedAsRemoved(_entity, type))
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
  this->EachNew([this, &stateMsg](const Entity &e) {
    this->AddEntityToMessage(stateMsg, e);
    return true;
  });
  this->EachRemoved([this, &stateMsg](const Entity &e) {
    this->AddEntityToMessage(stateMsg, e);
    return true;
  });
  this->registry.view<const ModifiedComponent>(entt::exclude<NewEntity, RemoveEntity>).each([this, &stateMsg](const Entity& e, const ModifiedComponent&) {
      this->AddEntityToMessage(stateMsg, e);
  });
  return stateMsg;
}

//////////////////////////////////////////////////
void EntityComponentManager::ChangedState(
    msgs::SerializedStateMap &_state) const
{
  // New entities
  this->EachNew([this, &_state](const Entity &e) {
    this->AddEntityToMessage(_state, e);
    return true;
  });
  this->EachRemoved([this, &_state](const Entity &e) {
    this->AddEntityToMessage(_state, e);
    return true;
  });
  this->registry.view<const ModifiedComponent>(entt::exclude<NewEntity, RemoveEntity>).each([this, &_state](const Entity& e, const ModifiedComponent&) {
      this->AddEntityToMessage(_state, e);
  });
}

//////////////////////////////////////////////////
void EntityComponentManagerPrivate::CalculateStateThreadLoad(const entt::basic_registry<Entity>& _registry)
{

  /*
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
  */
}

//////////////////////////////////////////////////
msgs::SerializedState EntityComponentManager::State(
    const std::unordered_set<Entity> &_entities,
    const std::unordered_set<ComponentTypeId> &_types) const
{
  msgs::SerializedState stateMsg;
  this->registry.view<const Entity>().each([this, &_entities, &stateMsg, &_types](const Entity& e) {
    if (_entities.empty() || _entities.find(e) != _entities.end())
    {
      this->AddEntityToMessage(stateMsg, e, _types);
    }
  });

  return stateMsg;
}

//////////////////////////////////////////////////
void EntityComponentManager::State(
    msgs::SerializedStateMap  &_state,
    const std::unordered_set<Entity> &_entities,
    const std::unordered_set<ComponentTypeId> &_types,
    bool _full) const
{
  this->registry.view<Entity>().each([&](const Entity& e) {
    if (!_entities.empty() && _entities.find(e) == _entities.end())
      return;
    this->AddEntityToMessage(_state, e, _types, _full);
  });
  return;

  // TODO(luca) Benchmark whether multithreaded state really helps
  /*
  std::mutex stateMapMutex;
  std::vector<std::thread> workers;

  const auto entityView = this->registry.view<Entity>();
  int numEntities = entityView.size();
  const auto addN = entityView.begin() + 100;

  // Set the number of threads to spawn to the min of the calculated thread
  // count or max threads that the hardware supports
  int maxThreads = std::thread::hardware_concurrency();
  uint64_t numThreads = std::min(numEntities, maxThreads);

  int entitiesPerThread = static_cast<int>(std::ceil(
    static_cast<double>(numEntities) / numThreads));

  auto functor = [&](auto itStart, auto itEnd)
  {
    msgs::SerializedStateMap threadMap;
    while (itStart != itEnd)
    {
      auto entity = *itStart;
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

  auto itBegin = entityView.begin();
  for (uint64_t i = 0; i < numThreads; ++i)
  {
    // If we have added all of the entities to the iterator vector, we are
    // done so push back the end iterator
    numEntities -= entitiesPerThread;
    auto itEnd = entityView.begin() + (i + 1) * entitiesPerThread;
    if (numEntities <= 0)
    {
      itEnd = entityView.end();
    }
    workers.push_back(std::thread(functor,
        itBegin, itEnd));
    itBegin = itEnd;
  }

  // Wait for each thread to finish processing its components
  std::for_each(workers.begin(), workers.end(), [](std::thread &_t)
  {
    _t.join();
  });
  */
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

      const components::BaseComponent *comp =
        this->ComponentImplementation(entity, typeId);

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
      this->dataPtr->CreateEntityImplementation(this->registry, entity);
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
        this->dataPtr->AddModifiedComponent(this->registry, entity);
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
      this->dataPtr->CreateEntityImplementation(this->registry, entity);
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
  // TODO(luca) consider doing a cache
  std::unordered_set<Entity> descendants;

  if (!this->HasEntity(_entity))
    return descendants;

  this->dataPtr->InsertEntityRecursive(this->registry, _entity, descendants);
  return descendants;
}

//////////////////////////////////////////////////
void EntityComponentManager::SetAllComponentsUnchanged()
{
  this->dataPtr->periodicChangedComponents.clear();
  this->dataPtr->oneTimeChangedComponents.clear();
  this->registry.clear<ModifiedComponent>();
}

/////////////////////////////////////////////////
void EntityComponentManager::SetChanged(
    const Entity _entity, const ComponentTypeId _type,
    sim::ComponentState _c)
{
  // make sure _entity exists
  if (!this->HasEntity(_entity))
    return;

  // make sure the entity has a component of type _type
  if (this->dataPtr->ComponentMarkedAsRemoved(_entity, _type))
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

  this->dataPtr->AddModifiedComponent(this->registry, _entity);
}

/////////////////////////////////////////////////
std::unordered_set<ComponentTypeId> EntityComponentManager::ComponentTypes(
    const Entity _entity) const
{
  std::unordered_set<ComponentTypeId> result;
  auto handle = entt::basic_handle<const decltype(this->registry)>(this->registry, _entity);
  for (const auto [typeId, storage] : handle.storage())
  {
    // TODO(luca) remember to add component types here for internal components
    if (typeId == entt::type_hash<NewEntity>::value() ||
        typeId == entt::type_hash<RemoveEntity>::value() ||
        typeId == entt::type_hash<ModifiedComponent>::value() ||
        typeId == entt::type_hash<PinnedEntity>::value() ||
        typeId == entt::type_hash<Children>::value())
    {
      continue;
    }
    result.insert(typeId);
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

/*
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

*/
/////////////////////////////////////////////////
void EntityComponentManagerPrivate::AddModifiedComponent(entt::basic_registry<Entity>& _registry, const Entity &_entity)
{
  if (_registry.any_of<NewEntity, RemoveEntity, ModifiedComponent>(_entity))
  {
    // modified component is already in newlyCreatedEntities,
    // toRemoveEntities list, or already marked as modified.
    return;
  }

  _registry.emplace<ModifiedComponent>(_entity);
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
  if (!this->HasEntity(_entity))
    return;
  if (_recursive)
  {
    std::unordered_set<Entity> toPin;
    this->dataPtr->InsertEntityRecursive(this->registry, _entity,
        toPin);
    for (const auto& e : toPin) {
      this->registry.emplace_or_replace<PinnedEntity>(e);
    }
  }
  else
  {
    this->registry.emplace_or_replace<PinnedEntity>(_entity);
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::UnpinEntity(const Entity _entity, bool _recursive)
{
  if (!this->HasEntity(_entity))
    return;
  if (_recursive)
  {
    std::unordered_set<Entity> toUnpin;
    this->dataPtr->InsertEntityRecursive(this->registry, _entity,
        toUnpin);
    for (const auto& e : toUnpin) {
      this->registry.remove<PinnedEntity>(e);
    }
  }
  else
  {
    this->registry.remove<PinnedEntity>(_entity);
  }
}

/////////////////////////////////////////////////
void EntityComponentManager::UnpinAllEntities()
{
  this->registry.clear<PinnedEntity>();
}

/////////////////////////////////////////////////
void EntityComponentManager::CopyFrom(const EntityComponentManager &_fromEcm)
{
  this->dataPtr->CopyFrom(*_fromEcm.dataPtr);
  const auto& newStorage = _fromEcm.registry.storage<NewEntity>();
  const auto& removeStorage = _fromEcm.registry.storage<RemoveEntity>();
  const auto& pinnedStorage = _fromEcm.registry.storage<PinnedEntity>();
  const auto& modifiedStorage = _fromEcm.registry.storage<ModifiedComponent>();
  const auto& childrenStorage = _fromEcm.registry.storage<Children>();
  // TODO(luca) clean this up by just copying entities through iterators
  // instead of using high level APIs such as CreateEntityImplementation
  _fromEcm.registry.view<const Entity>().each([&](const Entity& e) {
    if (this->HasEntity(e))
      this->registry.destroy(e);
    this->dataPtr->CreateEntityImplementation(this->registry, e);
    // Created entities already have a NewEntity component
    // Some storages might not be initialized
    if (!newStorage->contains(e))
      this->registry.remove<NewEntity>(e);
    if (removeStorage && removeStorage->contains(e))
      this->registry.emplace<RemoveEntity>(e);
    if (pinnedStorage && pinnedStorage->contains(e))
      this->registry.emplace<PinnedEntity>(e);
    if (modifiedStorage && modifiedStorage->contains(e))
      this->registry.emplace<ModifiedComponent>(e);
    // Children is already present
    if (childrenStorage->contains(e))
      this->registry.replace<Children>(e, childrenStorage->get(e));
    // Now copy the actual gazebo components
    const auto componentTypes = _fromEcm.ComponentTypes(e);
    for (const auto& typeId : componentTypes) {
      const auto& storage = _fromEcm.registry.storage(typeId);
      if (!storage) {
        gzwarn << "Storage not found for component, this shouldn't happen" << std::endl;
        continue;
      }
      const auto* baseCompPtr = static_cast<const components::BaseComponent*>(storage->value(e));
      if (!baseCompPtr) {
        gzwarn << "Couldn't cast component to its base class, this shouldn't happen" << std::endl;
        continue;
      }
      if (!components::Factory::Instance()->SyncComponent(this->registry, e, typeId, baseCompPtr->Clone().get()))
      {
        gzerr << "Failed syncing component!" << std::endl;
      }
    }
  });
}

/////////////////////////////////////////////////
EntityComponentManagerDiff EntityComponentManager::ComputeEntityDiff(
    const EntityComponentManager &_other) const
{
  EntityComponentManagerDiff diff;
  _other.registry.view<Entity>().each([&](const Entity& e) {
    if (!this->HasEntity(e))
      diff.InsertAddedEntity(e);
  });
  this->registry.view<Entity>().each([&](const Entity& e) {
    if (!_other.HasEntity(e))
      diff.InsertRemovedEntity(e);
  });
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
      this->dataPtr->CreateEntityImplementation(this->registry, entity);
      if (entity >= this->dataPtr->entityCount)
      {
        this->dataPtr->entityCount = entity;
      }
      copyComponents(entity);
      this->SetParentEntity(entity, _other.ParentEntity(entity));
    }
  }

  const auto& newStorage = _other.registry.storage<NewEntity>();
  const auto& removeStorage = _other.registry.storage<RemoveEntity>();
  const auto& pinnedStorage = _other.registry.storage<PinnedEntity>();
  const auto& modifiedStorage = _other.registry.storage<ModifiedComponent>();
  const auto& childrenStorage = _other.registry.storage<Children>();

  for (const auto &entity : _diff.RemovedEntities())
  {
    // if the entity is not in this ECM, add it before requesting for its
    // removal.
    if (!this->HasEntity(entity))
    {
      this->dataPtr->CreateEntityImplementation(this->registry, entity);
      // We want to set this entity as "removed", but
      // CreateEntityImplementation sets it as "newlyCreated",
      // so remove it from that list.
      this->registry.remove<NewEntity>(entity);
      // Copy components so that EachRemoved match correctly
      if (entity >= this->dataPtr->entityCount)
      {
        this->dataPtr->entityCount = entity;
      }
      copyComponents(entity);
      if (!newStorage->contains(entity))
        this->registry.remove<NewEntity>(entity);
      if (removeStorage && removeStorage->contains(entity))
        this->registry.emplace<RemoveEntity>(entity);
      if (pinnedStorage && pinnedStorage->contains(entity))
        this->registry.emplace<PinnedEntity>(entity);
      if (modifiedStorage && modifiedStorage->contains(entity))
        this->registry.emplace<ModifiedComponent>(entity);
      // Children is already present
      if (childrenStorage->contains(entity))
        this->registry.replace<Children>(entity, childrenStorage->get(entity));
      // TODO(luca) Any other components to copy? I.e. pin, modified component
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
  // TODO(luca) Just do a hashmap check with builtin name support
  std::optional<Entity> entity;
   Entity entByName = EntityByComponents(components::Name(_name));
  if (entByName != kNullEntity)
    entity = entByName;

  return entity;
}

/////////////////////////////////////////////////
void EntityComponentManager::MarkComponentAsRemoved(const Entity& _entity, const ComponentTypeId _id, bool _removed)
{
  if (_removed)
    this->dataPtr->componentsMarkedAsRemoved[_entity].insert(_id);
  else
    this->dataPtr->componentsMarkedAsRemoved[_entity].erase(_id);
}
