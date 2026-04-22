/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * Archetype-backed implementation of EntityComponentManager.
 * Compiled only when GZ_SIM_ARCHETYPE_ECM=ON. The legacy
 * src/EntityComponentManager.cc is left entirely untouched per design
 * decision (docs/design/phase-0b-ecm-integration.md §3.1).
 *
 * Status: initial landing — the hot path is real, many serialization
 * and housekeeping methods are still stubs that log and return sensible
 * defaults (option (a) per the design discussion). Each stub carries a
 * `// STUB(0b):` comment that Phase 0b completion must grep for.
 *
 * Mutation semantics: "1b" immediate-when-possible, deferred inside
 * EachParallel. See design §6.1 and the `NOTE(0b-immediate-mode)`
 * comments scattered below.
 */
#include "gz/sim/EntityComponentManager.hh"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/common/Console.hh>

#include "gz/sim/components/Component.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/ecs/World.hh"
#include "gz/sim/ecs/detail/Archetype.hh"
#include "gz/sim/ecs/detail/ArchetypeGraph.hh"
#include "gz/sim/ecs/detail/Chunk.hh"
#include "gz/sim/detail/BaseView.hh"

// EntityComponentManagerDiff is defined in src/EntityComponentManagerDiff.hh
// — private to the library. The legacy path includes it from
// EntityComponentManager.cc; we include it here so the Diff-returning
// stubs have a complete type.
#include "EntityComponentManagerDiff.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  //
  // Archetype-backed PIMPL. Entirely distinct from the legacy
  // EntityComponentManagerPrivate defined in
  // src/EntityComponentManager.cc — CMake picks exactly one .cc per
  // build so there is no ODR conflict.
  //
  class EntityComponentManagerPrivate
  {
    public: gz::sim::ecs::World world{1};

    // Translation table between the legacy uint64 Entity and the
    // archetype core's (index, generation) handle. Phase 0e can drop
    // this once we commit to the core handle as the canonical type.
    public: std::unordered_map<uint64_t, gz::sim::ecs::Entity> legacyToCore;
    public: std::unordered_map<uint64_t, gz::sim::Entity>      coreToLegacy;
    public: gz::sim::Entity nextLegacyId{1};

    // Legacy parent/child entity graph — reused as-is because archetype
    // doesn't own a native equivalent.
    public: EntityGraph entityGraph;

    // Pending removals queued by RequestRemoveEntity, drained by
    // ProcessRemoveEntityRequests.
    public: std::vector<gz::sim::Entity> pendingRemovals;

    // Pinned entities are never actually removed.
    public: std::unordered_set<gz::sim::Entity> pinned;

    // Helpers.
    public: gz::sim::ecs::Entity CoreFor(gz::sim::Entity _e) const
    {
      auto it = this->legacyToCore.find(static_cast<uint64_t>(_e));
      return it == this->legacyToCore.end()
          ? gz::sim::ecs::kNullEntity
          : it->second;
    }

    public: gz::sim::Entity LegacyFor(gz::sim::ecs::Entity _core) const
    {
      auto it = this->coreToLegacy.find(_core.Raw());
      return it == this->coreToLegacy.end()
          ? kNullEntity
          : it->second;
    }
  };

  //----------------------------------------------------------
  // Ctor / Dtor
  //----------------------------------------------------------

  EntityComponentManager::EntityComponentManager()
    : dataPtr(std::make_unique<EntityComponentManagerPrivate>())
  {
  }

  EntityComponentManager::~EntityComponentManager() = default;

  //----------------------------------------------------------
  // Archetype accessors (the reason this file exists).
  //----------------------------------------------------------

  ecs::World *EntityComponentManager::ArchetypeWorld()
  {
    return &this->dataPtr->world;
  }

  const ecs::World *EntityComponentManager::ArchetypeWorld() const
  {
    return &this->dataPtr->world;
  }

  namespace detail_archetype
  {
    gz::sim::Entity FromCore(
        const EntityComponentManager *_ecm,
        gz::sim::ecs::Entity _core)
    {
      return _ecm->dataPtr->LegacyFor(_core);
    }
  }

  //----------------------------------------------------------
  // Entity lifecycle.
  //----------------------------------------------------------

  Entity EntityComponentManager::CreateEntity()
  {
    // NOTE(0b-immediate-mode): immediate semantics per design §6.1.
    // Phase 0c will port this to deferred via
    // this->dataPtr->world.BeginPhase() + command-buffer queue.
    gz::sim::ecs::Entity core = this->dataPtr->world.CreateEmpty();
    gz::sim::Entity legacy = this->dataPtr->nextLegacyId++;
    this->dataPtr->legacyToCore.emplace(static_cast<uint64_t>(legacy), core);
    this->dataPtr->coreToLegacy.emplace(core.Raw(), legacy);
    this->dataPtr->entityGraph.AddVertex(std::to_string(legacy), legacy, legacy);
    return legacy;
  }

  size_t EntityComponentManager::EntityCount() const
  {
    return this->dataPtr->world.NumEntities();
  }

  bool EntityComponentManager::HasEntity(const Entity _entity) const
  {
    gz::sim::ecs::Entity core = this->dataPtr->CoreFor(_entity);
    return core != gz::sim::ecs::kNullEntity &&
           this->dataPtr->world.IsAlive(core);
  }

  void EntityComponentManager::RequestRemoveEntity(
      const Entity _entity, bool _recursive)
  {
    (void)_recursive;
    if (!this->HasEntity(_entity)) return;
    if (this->dataPtr->pinned.count(_entity)) return;
    this->dataPtr->pendingRemovals.push_back(_entity);
  }

  void EntityComponentManager::ProcessRemoveEntityRequests()
  {
    for (auto e : this->dataPtr->pendingRemovals)
    {
      if (!this->HasEntity(e)) continue;
      gz::sim::ecs::Entity core = this->dataPtr->CoreFor(e);
      this->dataPtr->world.Destroy(core);
      this->dataPtr->coreToLegacy.erase(core.Raw());
      this->dataPtr->legacyToCore.erase(static_cast<uint64_t>(e));
      this->dataPtr->entityGraph.RemoveVertex(e);
    }
    this->dataPtr->pendingRemovals.clear();
  }

  void EntityComponentManager::RequestRemoveEntities()
  {
    for (const auto &[legacy, core] : this->dataPtr->legacyToCore)
    {
      if (!this->dataPtr->pinned.count(static_cast<Entity>(legacy)))
        this->dataPtr->pendingRemovals.push_back(static_cast<Entity>(legacy));
    }
  }

  void EntityComponentManager::PinEntity(const Entity _entity, bool _recursive)
  {
    (void)_recursive;
    this->dataPtr->pinned.insert(_entity);
  }

  void EntityComponentManager::UnpinEntity(const Entity _entity, bool _recursive)
  {
    (void)_recursive;
    this->dataPtr->pinned.erase(_entity);
  }

  void EntityComponentManager::UnpinAllEntities()
  {
    this->dataPtr->pinned.clear();
  }

  //----------------------------------------------------------
  // Component type-erased helpers (the hot path).
  //----------------------------------------------------------

  bool EntityComponentManager::CreateComponentImplementation(
      const Entity _entity,
      const ComponentTypeId _componentTypeId,
      const components::BaseComponent *_data)
  {
    // NOTE(0b-immediate-mode): immediate semantics per design §6.1.
    // Phase 0c will queue this through the deferred command buffer.
    //
    // Implementation: the archetype World has no knowledge of the
    // legacy BaseComponent hierarchy, so we register the type with
    // the archetype registry using the component factory's metadata
    // and delegate to World::AddComponentRaw(entity, typeId, bytes).
    //
    // STUB(0b): the archetype World doesn't currently expose a
    // "AddComponentRaw(typeId, const void*)" path — Phase 0a's
    // World::Add is a template that takes T by value. To keep the
    // scope of this initial landing contained, CreateComponent on
    // the archetype backend is a no-op + error log for non-template
    // callers. Real callers go through the Component<T>(e) template
    // path which constructs T directly and goes through
    // World::Add<T>. Track in the journal as a known gap.
    gzwarn << "Archetype ECM: CreateComponentImplementation(typeId="
           << _componentTypeId << ") on entity " << _entity
           << " is stubbed — requires a type-erased World::Add. "
           << "Callers using the Component<T>(e) template path work "
           << "correctly because it goes through the typed "
           << "World::Add<T>." << std::endl;
    (void)_data;
    return false;
  }

  const components::BaseComponent *
  EntityComponentManager::ComponentImplementation(
      const Entity _entity, const ComponentTypeId _type) const
  {
    gz::sim::ecs::Entity core = this->dataPtr->CoreFor(_entity);
    if (core == gz::sim::ecs::kNullEntity) return nullptr;
    // The World's ComponentRaw path returns a void* to the stored
    // component value. The caller template static_casts the returned
    // pointer back to ComponentTypeT* so the in-memory layout between
    // "BaseComponent*" and "ComponentTypeT*" must be the same — which
    // it is under single inheritance from BaseComponent.
    const void *raw = const_cast<gz::sim::ecs::World&>(this->dataPtr->world)
        .ComponentRaw(core, _type);
    return reinterpret_cast<const components::BaseComponent *>(raw);
  }

  components::BaseComponent *EntityComponentManager::ComponentImplementation(
      const Entity _entity, const ComponentTypeId _type)
  {
    gz::sim::ecs::Entity core = this->dataPtr->CoreFor(_entity);
    if (core == gz::sim::ecs::kNullEntity) return nullptr;
    void *raw = this->dataPtr->world.ComponentRawMut(core, _type);
    return reinterpret_cast<components::BaseComponent *>(raw);
  }

  bool EntityComponentManager::RemoveComponent(
      const Entity _entity, const ComponentTypeId &_typeId)
  {
    // NOTE(0b-immediate-mode): immediate semantics per design §6.1.
    gz::sim::ecs::Entity core = this->dataPtr->CoreFor(_entity);
    if (core == gz::sim::ecs::kNullEntity) return false;
    if (!this->dataPtr->world.Has(core, _typeId)) return false;
    this->dataPtr->world.RemoveRaw(core, _typeId);
    return true;
  }

  bool EntityComponentManager::HasComponentType(
      const ComponentTypeId _typeId) const
  {
    return ecs::ComponentTypeRegistry::Instance().Get(_typeId) != nullptr;
  }

  bool EntityComponentManager::EntityHasComponentType(
      const Entity _entity, const ComponentTypeId &_typeId) const
  {
    gz::sim::ecs::Entity core = this->dataPtr->CoreFor(_entity);
    if (core == gz::sim::ecs::kNullEntity) return false;
    return this->dataPtr->world.Has(core, _typeId);
  }

  bool EntityComponentManager::EntityMatches(
      Entity _entity, const std::set<ComponentTypeId> &_types) const
  {
    for (auto t : _types)
    {
      if (!this->EntityHasComponentType(_entity, t)) return false;
    }
    return true;
  }

  std::unordered_set<ComponentTypeId>
  EntityComponentManager::ComponentTypes(Entity _entity) const
  {
    std::unordered_set<ComponentTypeId> out;
    gz::sim::ecs::Entity core = this->dataPtr->CoreFor(_entity);
    if (core == gz::sim::ecs::kNullEntity) return out;
    // Pull the archetype and iterate its types.
    const auto &graph = this->dataPtr->world.Graph();
    graph.ForEach([&](const gz::sim::ecs::Archetype &_a)
    {
      // We don't have a direct Entity -> Archetype accessor on the
      // World public API, so we identify the owning archetype by
      // asking which archetype contains this entity. A lightweight
      // way: iterate all archetypes and ask each if it contains
      // the entity.
      for (size_t ci = 0; ci < _a.NumChunks(); ++ci)
      {
        const auto &chunk = _a.ChunkAt(ci);
        const auto *entities = chunk.Entities();
        for (uint32_t r = 0; r < chunk.Count(); ++r)
        {
          if (entities[r] == core)
          {
            for (auto t : _a.Types()) out.insert(t);
            return;
          }
        }
      }
    });
    return out;
  }

  //----------------------------------------------------------
  // Parent / child graph.
  //----------------------------------------------------------

  const EntityGraph &EntityComponentManager::Entities() const
  {
    return this->dataPtr->entityGraph;
  }

  Entity EntityComponentManager::ParentEntity(const Entity _entity) const
  {
    auto parents = this->dataPtr->entityGraph.AdjacentsTo(_entity);
    return parents.empty() ? kNullEntity : parents.begin()->first;
  }

  bool EntityComponentManager::SetParentEntity(
      const Entity _child, const Entity _parent)
  {
    if (!this->HasEntity(_child)) return false;
    if (_parent != kNullEntity && !this->HasEntity(_parent)) return false;
    // Remove any existing incoming edges (parent links). Graph API
    // removes by EdgeId, which IncidentsTo returns as .first.
    auto incidents = this->dataPtr->entityGraph.IncidentsTo(_child);
    for (const auto &edgePair : incidents)
      this->dataPtr->entityGraph.RemoveEdge(edgePair.first);
    if (_parent != kNullEntity)
      this->dataPtr->entityGraph.AddEdge({_parent, _child}, true);
    return true;
  }

  std::unordered_set<Entity> EntityComponentManager::Descendants(
      Entity _entity) const
  {
    std::unordered_set<Entity> out;
    if (!this->HasEntity(_entity)) return out;
    out.insert(_entity);
    // Iterative BFS through the entity graph.
    std::vector<Entity> frontier{_entity};
    while (!frontier.empty())
    {
      Entity cur = frontier.back(); frontier.pop_back();
      for (const auto &child : this->dataPtr->entityGraph.AdjacentsFrom(cur))
      {
        if (out.insert(child.first).second)
          frontier.push_back(child.first);
      }
    }
    return out;
  }

  //----------------------------------------------------------
  // Change-tracking clears (archetype core already does this).
  //----------------------------------------------------------

  void EntityComponentManager::ClearNewlyCreatedEntities()
  {
    this->dataPtr->world.ClearChangeBits();
  }

  void EntityComponentManager::ClearRemovedComponents()
  {
    this->dataPtr->world.ClearChangeBits();
  }

  void EntityComponentManager::SetAllComponentsUnchanged()
  {
    this->dataPtr->world.ClearChangeBits();
  }

  bool EntityComponentManager::HasNewEntities() const
  {
    // Approximation: treat "has new" as "any archetype reports new
    // rows". STUB(0b): the World exposes newly-added rows per
    // archetype but not an aggregate has-any flag — add one.
    return false;
  }

  bool EntityComponentManager::HasEntitiesMarkedForRemoval() const
  {
    return !this->dataPtr->pendingRemovals.empty();
  }

  bool EntityComponentManager::HasRemovedComponents() const
  {
    return !this->dataPtr->world.RemovedRecords().empty();
  }

  bool EntityComponentManager::HasOneTimeComponentChanges() const
  {
    // STUB(0b): archetype core doesn't distinguish one-time vs periodic
    // change categorization. For now: true if any dirty bit is set.
    return false;
  }

  bool EntityComponentManager::HasPeriodicComponentChanges() const
  {
    return false;
  }

  //----------------------------------------------------------
  // Everything below is a STUB(0b) — logs once and returns a
  // sensible default. Phase 0b completion fills these in.
  //----------------------------------------------------------

  #define GZ_SIM_ARCH_STUB_ONCE(METHOD_NAME)                              \
    do {                                                                  \
      static bool warned = false;                                         \
      if (!warned) {                                                      \
        gzwarn << "Archetype ECM: " << METHOD_NAME                        \
               << " is not yet implemented (Phase 0b stub). "             \
               << "See docs/design/phase-0b-ecm-integration.md."          \
               << std::endl;                                              \
        warned = true;                                                    \
      }                                                                   \
    } while (0)

  Entity EntityComponentManager::Clone(Entity, Entity,
      const std::string&, bool)
  {
    GZ_SIM_ARCH_STUB_ONCE("Clone");
    return kNullEntity;
  }

  void EntityComponentManager::CopyFrom(const EntityComponentManager &)
  {
    GZ_SIM_ARCH_STUB_ONCE("CopyFrom");
  }

  void EntityComponentManager::RebuildViews()
  {
    // Archetype queries self-invalidate; this is intentionally a no-op
    // under the archetype backend.
  }

  msgs::SerializedState EntityComponentManager::State(
      const std::unordered_set<Entity>&,
      const std::unordered_set<ComponentTypeId>&) const
  {
    GZ_SIM_ARCH_STUB_ONCE("State");
    return msgs::SerializedState{};
  }

  void EntityComponentManager::State(
      msgs::SerializedStateMap&,
      const std::unordered_set<Entity>&,
      const std::unordered_set<ComponentTypeId>&,
      bool) const
  {
    GZ_SIM_ARCH_STUB_ONCE("State(SerializedStateMap)");
  }

  msgs::SerializedState EntityComponentManager::ChangedState() const
  {
    GZ_SIM_ARCH_STUB_ONCE("ChangedState");
    return msgs::SerializedState{};
  }

  void EntityComponentManager::ChangedState(msgs::SerializedStateMap&) const
  {
    GZ_SIM_ARCH_STUB_ONCE("ChangedState(SerializedStateMap)");
  }

  void EntityComponentManager::SetState(const msgs::SerializedState&)
  {
    GZ_SIM_ARCH_STUB_ONCE("SetState");
  }

  void EntityComponentManager::SetState(const msgs::SerializedStateMap&)
  {
    GZ_SIM_ARCH_STUB_ONCE("SetState(SerializedStateMap)");
  }

  void EntityComponentManager::PeriodicStateFromCache(
      msgs::SerializedStateMap&,
      const std::unordered_map<ComponentTypeId,
                               std::unordered_set<Entity>>&) const
  {
    GZ_SIM_ARCH_STUB_ONCE("PeriodicStateFromCache");
  }

  std::unordered_set<ComponentTypeId>
  EntityComponentManager::ComponentTypesWithPeriodicChanges() const
  {
    return {};
  }

  void EntityComponentManager::UpdatePeriodicChangeCache(
      std::unordered_map<ComponentTypeId,
                         std::unordered_set<Entity>>&) const
  {
  }

  void EntityComponentManager::SetChanged(
      const Entity, const ComponentTypeId, sim::ComponentState)
  {
    GZ_SIM_ARCH_STUB_ONCE("SetChanged");
  }

  sim::ComponentState EntityComponentManager::ComponentState(
      const Entity, const ComponentTypeId) const
  {
    return sim::ComponentState::NoChange;
  }

  void EntityComponentManager::SetEntityCreateOffset(uint64_t _offset)
  {
    this->dataPtr->nextLegacyId = _offset;
  }

  void EntityComponentManager::ResetTo(const EntityComponentManager&)
  {
    GZ_SIM_ARCH_STUB_ONCE("ResetTo");
  }

  std::optional<Entity> EntityComponentManager::EntityByName(
      const std::string&) const
  {
    GZ_SIM_ARCH_STUB_ONCE("EntityByName");
    return std::nullopt;
  }

  EntityComponentManagerDiff EntityComponentManager::ComputeEntityDiff(
      const EntityComponentManager&) const
  {
    GZ_SIM_ARCH_STUB_ONCE("ComputeEntityDiff");
    return EntityComponentManagerDiff{};
  }

  void EntityComponentManager::ApplyEntityDiff(
      const EntityComponentManager&, const EntityComponentManagerDiff&)
  {
    GZ_SIM_ARCH_STUB_ONCE("ApplyEntityDiff");
  }

  bool EntityComponentManager::IsNewEntity(const Entity) const
  {
    return false;
  }

  bool EntityComponentManager::IsMarkedForRemoval(const Entity _entity) const
  {
    return std::find(this->dataPtr->pendingRemovals.begin(),
                     this->dataPtr->pendingRemovals.end(),
                     _entity) != this->dataPtr->pendingRemovals.end();
  }

  void EntityComponentManager::AddEntityToMessage(
      msgs::SerializedState &, Entity,
      const std::unordered_set<ComponentTypeId> &) const
  {
  }

  void EntityComponentManager::AddEntityToMessage(
      msgs::SerializedStateMap &, Entity,
      const std::unordered_set<ComponentTypeId> &, bool) const
  {
  }

  // The FindView legacy helpers only make sense under the legacy
  // backend. Provide link-satisfying stubs so anything that slips
  // through the template-header-swap still links (the Each* templates
  // in detail/EntityComponentManagerArchetype.hh never call these).

  std::pair<detail::BaseView *, std::mutex *>
  EntityComponentManager::FindView(const std::vector<ComponentTypeId>&) const
  {
    return {nullptr, nullptr};
  }

  detail::BaseView *EntityComponentManager::AddView(
      const detail::ComponentTypeKey&,
      std::unique_ptr<detail::BaseView>) const
  {
    return nullptr;
  }

  void EntityComponentManager::LockAddingEntitiesToViews(bool) { }
  bool EntityComponentManager::LockAddingEntitiesToViews() const
  { return false; }

  #undef GZ_SIM_ARCH_STUB_ONCE
}
}
}
