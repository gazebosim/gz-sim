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

    // Newly-created entities — populated in CreateEntity /
    // SetState's create path, drained by ClearNewlyCreatedEntities
    // (invoked at end-of-step by SimulationRunner). EachNew<T>
    // filters against this set so systems only see each entity once.
    public: std::unordered_set<gz::sim::Entity> newlyCreatedEntities;

    // Per-typeId sets of entities whose component changed this step.
    // Mirrors legacy ECM's periodicChangedComponents /
    // oneTimeChangedComponents side tables. Populated by SetChanged,
    // drained by SetAllComponentsUnchanged.
    public: std::unordered_map<ComponentTypeId,
        std::unordered_set<gz::sim::Entity>> periodicChangedComponents;
    public: std::unordered_map<ComponentTypeId,
        std::unordered_set<gz::sim::Entity>> oneTimeChangedComponents;

    // Mirrors legacy `modifiedComponents`: entities whose components
    // changed THIS step *and* which weren't already covered by the
    // newlyCreatedEntities or pendingRemovals lists. Used by
    // ChangedState() to walk modified-but-not-new entities. Populated
    // via AddModifiedComponent(), drained on ClearAllChanged events.
    public: std::unordered_set<gz::sim::Entity> modifiedComponents;

    public: void AddModifiedComponent(gz::sim::Entity _e)
    {
      // Match legacy filter (src/EntityComponentManager.cc
      // AddModifiedComponent): suppress when the entity is already
      // covered by newly-created or pending-removal tracking.
      if (this->newlyCreatedEntities.count(_e) ||
          std::find(this->pendingRemovals.begin(),
                    this->pendingRemovals.end(), _e) !=
              this->pendingRemovals.end())
        return;
      this->modifiedComponents.insert(_e);
    }

    // Pattern B (Phase 0b test-failure analysis,
    // docs/design/0b-test-failures.md §2): legacy ECM defers actual
    // destruction of removed components until ClearRemovedComponents.
    // This keeps pointers obtained before RemoveComponent valid for
    // the rest of the step. Removed components move from
    // shadowComponents into removedComponentsPending; reads via
    // ComponentImplementation/EntityHasComponentType skip them, but
    // memory stays alive until the end-of-step drain.
    public: std::vector<std::unique_ptr<components::BaseComponent>>
        removedComponentsPending;
    // Per-(entity, typeId) set marking removed-but-not-yet-destroyed
    // shadow entries. Read paths use this to filter, plus
    // EachRemoved iterates it.
    public: std::unordered_map<gz::sim::Entity,
        std::unordered_set<ComponentTypeId>> componentsMarkedAsRemoved;

    // Set of typeIds that have ever been seen in this ECM. Mirrors
    // legacy ECM's createdCompTypes — backs HasComponentType.
    public: std::unordered_set<ComponentTypeId> createdCompTypes;

    // Working state for Clone(). Cleared at the top of the public
    // Clone entrypoint. CloneImpl() populates these so the post-clone
    // pass can fix up canonical-link and joint-link references on the
    // cloned entities. Lifetimes are scoped to one Clone() call.
    public: std::unordered_map<gz::sim::Entity, gz::sim::Entity>
        oldToClonedCanonicalLink;
    public: std::unordered_map<gz::sim::Entity, gz::sim::Entity>
        oldModelCanonicalLink;
    public: std::unordered_map<gz::sim::Entity, gz::sim::Entity>
        originalToClonedLink;
    public: std::unordered_map<gz::sim::Entity,
                               std::pair<gz::sim::Entity, gz::sim::Entity>>
        clonedToOriginalJointLinks;

    // Shadow component storage.
    //
    // The archetype core's `World::Add<T>` / `World::ComponentRaw<T>`
    // paths require the ComponentTypeRegistry to have an entry for T.
    // The entry is populated lazily on first call of `Register<T>()`,
    // which only fires when the template is instantiated with a
    // concrete T. That works fine for the template ECM paths
    // (`Component<T>(e)`, `CreateComponent<T>(e, T{...})`), but
    // SdfEntityCreator and the legacy ComponentFactory pipeline call
    // the *type-erased* `CreateComponentImplementation(e, typeId,
    // BaseComponent*)` path, where only the dynamic typeId and a
    // polymorphic BaseComponent pointer are known — we can't bridge
    // those into the archetype registry without a Factory-side bridge
    // that knows T statically for every registered component.
    //
    // For Phase 0b we sidestep this with a shadow map:
    // `shadowComponents[entity][typeId]` owns a `unique_ptr<BaseComponent>`
    // cloned from the caller's pointer. The non-template
    // ComponentImplementation() path checks here first; template
    // callers continue to go through the archetype World at full
    // performance.
    //
    // Performance note: components stored in the shadow map do NOT
    // appear in `World::Each<T>` iterations. Systems that rely on
    // `Each<T>` over components created by SdfEntityCreator (e.g.,
    // the first pass of Physics system enumeration) will miss those
    // entities. The fix — a Factory ↔ ComponentTypeRegistry bridge
    // that registers every GZ_SIM_REGISTER_COMPONENT type into the
    // archetype registry at load time — is the Phase 0b completion
    // item. Marker: NOTE(0b-shadow-store).
    public: std::unordered_map<gz::sim::Entity,
        std::unordered_map<ComponentTypeId,
            std::unique_ptr<components::BaseComponent>>> shadowComponents;

    // Helpers.
    //
    // NOTE: the archetype core's first-created entity has raw value 0
    // (index=0, gen=0), which happens to equal kNullEntity's raw bit
    // pattern. Using kNullEntity as a "not in map" sentinel from
    // CoreFor would misclassify the world entity (typically the first
    // one created). Instead, CoreFor returns a bool + out-param so
    // callers distinguish "not in map" from "in map, and the core
    // handle happens to be 0".
    public: bool CoreFor(gz::sim::Entity _e,
                         gz::sim::ecs::Entity &_out) const
    {
      auto it = this->legacyToCore.find(static_cast<uint64_t>(_e));
      if (it == this->legacyToCore.end()) return false;
      _out = it->second;
      return true;
    }

    public: gz::sim::Entity LegacyFor(gz::sim::ecs::Entity _core) const
    {
      auto it = this->coreToLegacy.find(_core.Raw());
      return it == this->coreToLegacy.end()
          ? kNullEntity
          : it->second;
    }

    /// \brief True if the entity has this component in the shadow map.
    public: bool ShadowHas(gz::sim::Entity _e, ComponentTypeId _t) const
    {
      auto eit = this->shadowComponents.find(_e);
      if (eit == this->shadowComponents.end()) return false;
      return eit->second.find(_t) != eit->second.end();
    }

    /// \brief Fetch from shadow map. Nullptr if absent.
    public: components::BaseComponent *ShadowGet(
        gz::sim::Entity _e, ComponentTypeId _t) const
    {
      auto eit = this->shadowComponents.find(_e);
      if (eit == this->shadowComponents.end()) return nullptr;
      auto cit = eit->second.find(_t);
      return cit == eit->second.end() ? nullptr : cit->second.get();
    }

    /// \brief Pattern B: a component was marked-removed this step.
    public: bool IsMarkedRemoved(
        gz::sim::Entity _e, ComponentTypeId _t) const
    {
      auto it = this->componentsMarkedAsRemoved.find(_e);
      if (it == this->componentsMarkedAsRemoved.end()) return false;
      return it->second.count(_t) > 0;
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

  std::vector<Entity>
  EntityComponentManager::AllEntitiesArchetypeFacade() const
  {
    // Iterate legacyToCore. The archetype World's entity set is a
    // subset of this map (every core entity has a legacy id mirror);
    // the shadow store's keys are also a subset. One pass covers
    // both storage paths.
    //
    // Return sorted ascending. gz-sim's entity-creation convention
    // is top-down: parents are created before children, so lower
    // legacy IDs are always parents of higher ones. Downstream
    // consumers — particularly the scene broadcaster's state
    // emission and the GUI's EntityTree plugin — rely on
    // parent-first arrival order to avoid a fragile recursive
    // pending-entities dance that can invalidate iterators on
    // arbitrary orderings. See docs/design/JOURNAL.md for the
    // 2026-04-24 EntityTree crash root-cause note.
    std::vector<Entity> out;
    out.reserve(this->dataPtr->legacyToCore.size());
    for (const auto &[k, _] : this->dataPtr->legacyToCore)
      out.push_back(static_cast<Entity>(k));
    std::sort(out.begin(), out.end());
    return out;
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
    this->dataPtr->newlyCreatedEntities.insert(legacy);
    return legacy;
  }

  size_t EntityComponentManager::EntityCount() const
  {
    return this->dataPtr->world.NumEntities();
  }

  bool EntityComponentManager::HasEntity(const Entity _entity) const
  {
    gz::sim::ecs::Entity core;
    if (!this->dataPtr->CoreFor(_entity, core)) return false;
    return this->dataPtr->world.IsAlive(core);
  }

  void EntityComponentManager::RequestRemoveEntity(
      const Entity _entity, bool _recursive)
  {
    // Legacy semantics: invalid entities still appear as
    // "marked for removal" — the test
    // EntityComponentManagerFixture.RemoveEntity asserts this. The
    // actual destruction happens (or silently no-ops for invalid IDs)
    // in ProcessRemoveEntityRequests.
    if (!this->HasEntity(_entity))
    {
      this->dataPtr->pendingRemovals.push_back(_entity);
      return;
    }

    if (_recursive)
    {
      // Pattern C: enqueue the entity and every descendant in the
      // entity graph. Pinning is honored — pinned entities are
      // skipped (and their descendants survive too, matching legacy
      // semantics where the pinned subtree is left intact).
      auto descs = this->Descendants(_entity);

      // Mirror legacy: detachable joints are orphan entities (no
      // parent in the entity graph) but they reference parent/child
      // link entities. When we recursively remove a model, also
      // sweep up any DetachableJoint whose endpoints are about to
      // disappear — otherwise the joints leak. See
      // src/EntityComponentManager.cc RequestRemoveEntity.
      this->Each<components::DetachableJoint>(
          [&](const Entity &_jointEntity,
              const components::DetachableJoint *_jointInfo) -> bool
          {
            const auto &d = _jointInfo->Data();
            if (descs.count(d.parentLink) || descs.count(d.childLink))
              descs.insert(_jointEntity);
            return true;
          });

      for (auto e : descs)
      {
        if (this->dataPtr->pinned.count(e)) continue;
        this->dataPtr->pendingRemovals.push_back(e);
      }
    }
    else
    {
      if (this->dataPtr->pinned.count(_entity)) return;
      this->dataPtr->pendingRemovals.push_back(_entity);
    }
  }

  void EntityComponentManager::ProcessRemoveEntityRequests()
  {
    for (auto e : this->dataPtr->pendingRemovals)
    {
      if (!this->HasEntity(e)) continue;
      gz::sim::ecs::Entity core;
      if (!this->dataPtr->CoreFor(e, core)) continue;
      this->dataPtr->world.Destroy(core);
      this->dataPtr->coreToLegacy.erase(core.Raw());
      this->dataPtr->legacyToCore.erase(static_cast<uint64_t>(e));
      this->dataPtr->entityGraph.RemoveVertex(e);
      this->dataPtr->shadowComponents.erase(e);
      this->dataPtr->newlyCreatedEntities.erase(e);
      this->dataPtr->modifiedComponents.erase(e);
      this->dataPtr->componentsMarkedAsRemoved.erase(e);
      for (auto &kv : this->dataPtr->periodicChangedComponents)
        kv.second.erase(e);
      for (auto &kv : this->dataPtr->oneTimeChangedComponents)
        kv.second.erase(e);
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
    if (_recursive)
    {
      for (auto e : this->Descendants(_entity))
        this->dataPtr->pinned.insert(e);
    }
    else
    {
      this->dataPtr->pinned.insert(_entity);
    }
  }

  void EntityComponentManager::UnpinEntity(const Entity _entity, bool _recursive)
  {
    if (_recursive)
    {
      for (auto e : this->Descendants(_entity))
        this->dataPtr->pinned.erase(e);
    }
    else
    {
      this->dataPtr->pinned.erase(_entity);
    }
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
    // NOTE(0b-shadow-store): type-erased callers (SdfEntityCreator,
    // ComponentFactory) land here without T available. We can't feed
    // them into the archetype World without a Factory↔ecs registry
    // bridge (Phase 0b completion). For now, clone the BaseComponent
    // and keep it in the per-ECM shadow map; the read path
    // (ComponentImplementation / EntityHasComponentType /
    // ComponentTypes) checks the shadow map first, so callers see
    // consistent state. Template callers (Component<T>(e)) still
    // transit the archetype World at full performance.
    if (!this->HasEntity(_entity)) return false;
    if (!_data) return false;

    // Track typeIds we've ever seen so HasComponentType() reports
    // correctly, mirroring legacy's createdCompTypes.
    this->dataPtr->createdCompTypes.insert(_componentTypeId);

    // If the type IS registered with the archetype registry (e.g.
    // because someone already called Component<T>(e) or Add<T> for
    // this T), prefer the archetype-native path so the data lands in
    // the World where Each<T> will find it.
    if (auto *info = ecs::ComponentTypeRegistry::Instance().Get(
            _componentTypeId))
    {
      gz::sim::ecs::Entity core;
      if (this->dataPtr->CoreFor(_entity, core))
      {
        this->dataPtr->world.AddRaw(core, _componentTypeId,
            reinterpret_cast<const void *>(_data));
        // Drop any shadow copy — archetype is authoritative now.
        auto eit = this->dataPtr->shadowComponents.find(_entity);
        if (eit != this->dataPtr->shadowComponents.end())
          eit->second.erase(_componentTypeId);
        (void)info;
        return false;  // data was stored; no further update needed.
      }
    }

    // Fallback: clone into the shadow map.
    auto clone = _data->Clone();
    if (!clone) return false;
    this->dataPtr->shadowComponents[_entity][_componentTypeId]
        = std::move(clone);

    // Pattern A (Phase 0b test-failure analysis,
    // docs/design/0b-test-failures.md §1): the legacy ECM auto-wires
    // an entity-graph edge when a ParentEntity component is created.
    // Many in-tree consumers (Joint::Sensors, Link::*, Model::Links,
    // World::ModelByName, Physics, etc.) rely on the graph for
    // parent/child queries. Mirror the legacy behavior so callers
    // that build hierarchy via CreateComponent<ParentEntity> see
    // the same effect.
    if (_componentTypeId == components::ParentEntity::typeId)
    {
      auto *parentComp =
          this->Component<components::ParentEntity>(_entity);
      if (parentComp)
        this->SetParentEntity(_entity, parentComp->Data());
    }

    // Mirror legacy CreateComponentImplementation
    // (src/EntityComponentManager.cc ~1125): every new component is
    // recorded as a one-time change for ComponentState() reporting,
    // and the entity is added to modifiedComponents (subject to the
    // newlyCreated/pendingRemoval filter inside AddModifiedComponent).
    this->dataPtr->oneTimeChangedComponents[_componentTypeId].insert(
        _entity);
    this->dataPtr->AddModifiedComponent(_entity);
    // Component re-added after a prior remove — clear the
    // "marked as removed" flag so ChangedState doesn't emit a
    // contradictory remove=true alongside the new value.
    auto rit =
        this->dataPtr->componentsMarkedAsRemoved.find(_entity);
    if (rit != this->dataPtr->componentsMarkedAsRemoved.end())
      rit->second.erase(_componentTypeId);
    return false;  // data was stored via clone; caller need not update.
  }

  const components::BaseComponent *
  EntityComponentManager::ComponentImplementation(
      const Entity _entity, const ComponentTypeId _type) const
  {
    // Shadow map first (see NOTE(0b-shadow-store) on the PIMPL).
    if (auto *p = this->dataPtr->ShadowGet(_entity, _type))
      return p;
    gz::sim::ecs::Entity core;
    if (!this->dataPtr->CoreFor(_entity, core)) return nullptr;
    const void *raw = const_cast<gz::sim::ecs::World&>(this->dataPtr->world)
        .ComponentRaw(core, _type);
    return reinterpret_cast<const components::BaseComponent *>(raw);
  }

  components::BaseComponent *EntityComponentManager::ComponentImplementation(
      const Entity _entity, const ComponentTypeId _type)
  {
    if (auto *p = this->dataPtr->ShadowGet(_entity, _type))
      return p;

    gz::sim::ecs::Entity core;
    if (!this->dataPtr->CoreFor(_entity, core)) return nullptr;
    void *raw = this->dataPtr->world.ComponentRawMut(core, _type);
    return reinterpret_cast<components::BaseComponent *>(raw);
  }

  bool EntityComponentManager::RemoveComponent(
      const Entity _entity, const ComponentTypeId &_typeId)
  {
    // NOTE(0b-immediate-mode): immediate semantics per design §6.1.
    bool removed = false;

    // Pattern B: shadow-store removals defer destruction. Move the
    // unique_ptr into removedComponentsPending so the caller's prior
    // BaseComponent* stays valid until ClearRemovedComponents.
    auto eit = this->dataPtr->shadowComponents.find(_entity);
    if (eit != this->dataPtr->shadowComponents.end())
    {
      auto cit = eit->second.find(_typeId);
      if (cit != eit->second.end())
      {
        this->dataPtr->removedComponentsPending.push_back(
            std::move(cit->second));
        eit->second.erase(cit);
        this->dataPtr->componentsMarkedAsRemoved[_entity].insert(
            _typeId);
        removed = true;
      }
    }

    // Mirror legacy: removing a component clears any change-tracking
    // for that (entity, type). Tests like SetChanged/SetRemoved* rely
    // on ComponentState() reverting to NoChange after a remove.
    auto pit = this->dataPtr->periodicChangedComponents.find(_typeId);
    if (pit != this->dataPtr->periodicChangedComponents.end())
      pit->second.erase(_entity);
    auto oit = this->dataPtr->oneTimeChangedComponents.find(_typeId);
    if (oit != this->dataPtr->oneTimeChangedComponents.end())
      oit->second.erase(_entity);

    // The entity has a structural change — surfacing it through
    // ChangedState() requires modifiedComponents, mirroring legacy
    // behavior at src/EntityComponentManager.cc RemoveComponent.
    if (removed) this->dataPtr->AddModifiedComponent(_entity);

    // Archetype world (template path) — destroyed immediately. The
    // per-cell BaseComponent layout in the archetype world doesn't
    // give us a non-template clone-out path today; pointers obtained
    // through World::ComponentRaw before this call are invalidated.
    // For Phase 0b this is acceptable because in-tree callers that
    // hold pointers across removes use the legacy CreateComponent
    // (shadow-store) path.
    gz::sim::ecs::Entity core;
    if (this->dataPtr->CoreFor(_entity, core) &&
        this->dataPtr->world.Has(core, _typeId))
    {
      this->dataPtr->world.RemoveRaw(core, _typeId);
      removed = true;
    }
    return removed;
  }

  bool EntityComponentManager::HasComponentType(
      const ComponentTypeId _typeId) const
  {
    // Mirror legacy: a type "exists" once we've seen it, even if no
    // entity currently has one.
    return this->dataPtr->createdCompTypes.count(_typeId) > 0;
  }

  bool EntityComponentManager::EntityHasComponentType(
      const Entity _entity, const ComponentTypeId &_typeId) const
  {
    // Shadow store first (non-template created components).
    if (this->dataPtr->ShadowHas(_entity, _typeId)) return true;
    gz::sim::ecs::Entity core;
    if (!this->dataPtr->CoreFor(_entity, core)) return false;
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
    // Shadow map contributions first.
    auto eit = this->dataPtr->shadowComponents.find(_entity);
    if (eit != this->dataPtr->shadowComponents.end())
    {
      for (const auto &[tid, _] : eit->second) out.insert(tid);
    }

    gz::sim::ecs::Entity core;
    if (!this->dataPtr->CoreFor(_entity, core)) return out;
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
    this->dataPtr->newlyCreatedEntities.clear();
    this->dataPtr->world.ClearChangeBits();
  }

  void EntityComponentManager::ClearRemovedComponents()
  {
    // Pattern B: drain the deferred-destruction list and the
    // marked-as-removed set. Until this point, BaseComponent
    // pointers handed out before the RemoveComponent call remain
    // valid.
    this->dataPtr->removedComponentsPending.clear();
    this->dataPtr->componentsMarkedAsRemoved.clear();
    this->dataPtr->world.ClearChangeBits();
  }

  void EntityComponentManager::SetAllComponentsUnchanged()
  {
    this->dataPtr->periodicChangedComponents.clear();
    this->dataPtr->oneTimeChangedComponents.clear();
    this->dataPtr->modifiedComponents.clear();
    this->dataPtr->world.ClearChangeBits();
  }

  void EntityComponentManager::BeginPhase()
  {
    // Phase 0c plumbing: the archetype core has full phase semantics
    // (BeginPhase / Commit on `ecs::World`). The facade still runs in
    // immediate-mutation mode while in-tree systems are audited (see
    // docs/design/phase-0c-system-port.md §4.0). When the 0e flip
    // lands, this becomes `this->dataPtr->world.BeginPhase()` and the
    // CreateComponent/RemoveComponent/SetComponentData paths route
    // through the World's command buffer. Until then, this is a
    // documented no-op so SimulationRunner's wiring is in place.
  }

  void EntityComponentManager::CommitPhase()
  {
    // Same: no-op until the 0e flip. Will become
    // `this->dataPtr->world.Commit()` plus a drain of any parallel
    // shadow-store deferred buffer.
  }

  bool EntityComponentManager::HasNewEntities() const
  {
    return !this->dataPtr->newlyCreatedEntities.empty();
  }

  bool EntityComponentManager::HasEntitiesMarkedForRemoval() const
  {
    return !this->dataPtr->pendingRemovals.empty();
  }

  bool EntityComponentManager::HasRemovedComponents() const
  {
    // Pattern B: components are removed-but-pending-destruction
    // until ClearRemovedComponents drains the list.
    return !this->dataPtr->removedComponentsPending.empty() ||
           !this->dataPtr->world.RemovedRecords().empty();
  }

  bool EntityComponentManager::HasOneTimeComponentChanges() const
  {
    for (const auto &[_, ents] : this->dataPtr->oneTimeChangedComponents)
      if (!ents.empty()) return true;
    return false;
  }

  bool EntityComponentManager::HasPeriodicComponentChanges() const
  {
    for (const auto &[_, ents] : this->dataPtr->periodicChangedComponents)
      if (!ents.empty()) return true;
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

  Entity EntityComponentManager::Clone(Entity _entity, Entity _parent,
      const std::string &_name, bool _allowRename)
  {
    auto *d = this->dataPtr.get();
    d->oldToClonedCanonicalLink.clear();
    d->oldModelCanonicalLink.clear();
    d->originalToClonedLink.clear();
    d->clonedToOriginalJointLinks.clear();

    // Recursive helper kept inside Clone() so it can call private
    // CreateComponentImplementation. Mirrors legacy CloneImpl in
    // src/EntityComponentManager.cc.
    std::function<Entity(Entity, Entity, const std::string&, bool)> cloneImpl;
    cloneImpl = [&](Entity ent, Entity parent, const std::string &name,
                    bool allowRename) -> Entity
    {
      bool uniqueNameGenerated = false;
      if (!this->HasEntity(ent))
      {
        gzerr << "Requested to clone entity [" << ent
          << "], but this entity does not exist." << std::endl;
        return kNullEntity;
      }
      if (!name.empty() && !allowRename)
      {
        auto origParentComp =
            this->Component<components::ParentEntity>(ent);
        Entity dup = this->EntityByComponents(components::Name(name),
            components::ParentEntity(
                origParentComp ? origParentComp->Data() : kNullEntity));
        bool hasRecreateComp = false;
        Entity rec = dup;
        while (rec != kNullEntity && !hasRecreateComp)
        {
          hasRecreateComp =
              this->Component<components::Recreate>(rec) != nullptr;
          auto p = this->Component<components::ParentEntity>(rec);
          rec = p ? p->Data() : kNullEntity;
        }
        if (kNullEntity != dup && !hasRecreateComp)
        {
          gzerr << "Requested to clone entity [" << ent
            << "] with a name of [" << name << "], but another entity already "
            << "has this name." << std::endl;
          return kNullEntity;
        }
        uniqueNameGenerated = true;
      }

      Entity clonedEntity = this->CreateEntity();
      if (parent != kNullEntity)
      {
        this->SetParentEntity(clonedEntity, parent);
        this->CreateComponent(clonedEntity, components::ParentEntity(parent));
      }

      std::string clonedName = name;
      if (!uniqueNameGenerated)
      {
        if (clonedName.empty())
        {
          auto orig = this->Component<components::Name>(ent);
          clonedName = orig ? orig->Data() : "cloned_entity";
        }
        uint64_t suffix = 1;
        while (kNullEntity != this->EntityByComponents(
              components::Name(clonedName + "_" + std::to_string(suffix))))
          suffix++;
        clonedName += "_" + std::to_string(suffix);
      }
      this->CreateComponent(clonedEntity, components::Name(clonedName));

      for (const auto &type : this->ComponentTypes(ent))
      {
        if (type == components::Name::typeId ||
            type == components::ParentEntity::typeId)
          continue;
        auto orig = this->ComponentImplementation(ent, type);
        if (!orig) continue;
        auto cloned = orig->Clone();
        if (!cloned) continue;
        this->CreateComponentImplementation(clonedEntity, type, cloned.get());
      }

      if (auto modelCanonLinkComp =
          this->Component<components::ModelCanonicalLink>(clonedEntity))
      {
        d->oldModelCanonicalLink[clonedEntity] = modelCanonLinkComp->Data();
      }
      else if (this->Component<components::CanonicalLink>(clonedEntity))
      {
        d->oldToClonedCanonicalLink[ent] = clonedEntity;
      }

      if (this->Component<components::Joint>(clonedEntity))
      {
        Entity originalParentLink = kNullEntity;
        Entity originalChildLink = kNullEntity;
        auto origParentComp =
            this->Component<components::ParentEntity>(ent);
        const auto &parentName =
            this->Component<components::ParentLinkName>(ent);
        if (parentName && origParentComp)
        {
          if (common::lowercase(parentName->Data()) == "world")
          {
            originalParentLink = this->Component<components::ParentEntity>(
                origParentComp->Data())->Data();
          }
          else
          {
            originalParentLink = this->EntityByComponents<
                components::Name, components::ParentEntity>(
                  components::Name(parentName->Data()),
                  components::ParentEntity(origParentComp->Data()));
          }
        }
        const auto &childName =
            this->Component<components::ChildLinkName>(ent);
        if (childName && origParentComp)
        {
          originalChildLink = this->EntityByComponents<
              components::Name, components::ParentEntity>(
                components::Name(childName->Data()),
                components::ParentEntity(origParentComp->Data()));
        }
        if (!originalParentLink || !originalChildLink)
        {
          gzerr << "The cloned joint entity [" << clonedEntity
            << "] was unable to find the original joint entity's "
            << "parent and/or child link." << std::endl;
          this->RequestRemoveEntity(clonedEntity);
          return kNullEntity;
        }
        d->clonedToOriginalJointLinks[clonedEntity] =
            {originalParentLink, originalChildLink};
      }
      else if (this->Component<components::Link>(clonedEntity) ||
               this->Component<components::CanonicalLink>(clonedEntity))
      {
        d->originalToClonedLink[ent] = clonedEntity;
      }

      for (const auto &childEntity :
          this->EntitiesByComponents(components::ParentEntity(ent)))
      {
        std::string n;
        if (!allowRename)
        {
          auto nameComp = this->Component<components::Name>(childEntity);
          n = nameComp ? nameComp->Data() : "";
        }
        auto clonedChild = cloneImpl(childEntity, clonedEntity, n,
            allowRename);
        if (kNullEntity == clonedChild)
        {
          gzerr << "Cloning child entity [" << childEntity << "] failed.\n";
          this->RequestRemoveEntity(clonedEntity);
          return kNullEntity;
        }
      }
      return clonedEntity;
    };

    auto clonedEntity = cloneImpl(_entity, _parent, _name, _allowRename);
    if (kNullEntity == clonedEntity) return clonedEntity;

    for (const auto &[clonedModel, oldCanonical] : d->oldModelCanonicalLink)
    {
      auto it = d->oldToClonedCanonicalLink.find(oldCanonical);
      if (it == d->oldToClonedCanonicalLink.end())
      {
        gzerr << "Error: attempted to clone model(s) with canonical link(s),"
          << " but entity [" << oldCanonical << "] was not cloned as a "
          << "canonical link." << std::endl;
        continue;
      }
      this->SetComponentData<components::ModelCanonicalLink>(
          clonedModel, it->second);
    }

    auto repaintLinkName = [&](auto componentTag, Entity clonedJoint,
                               Entity originalLink) -> bool
    {
      using Tag = decltype(componentTag);
      auto it = d->originalToClonedLink.find(originalLink);
      if (it == d->originalToClonedLink.end()) return false;
      auto *nameComp = this->Component<components::Name>(it->second);
      if (!nameComp) return false;
      this->SetComponentData<Tag>(clonedJoint, nameComp->Data());
      return true;
    };

    for (const auto &[clonedJoint, origLinks] : d->clonedToOriginalJointLinks)
    {
      if (!repaintLinkName(components::ParentLinkName{}, clonedJoint,
              origLinks.first))
      {
        gzerr << "Error updating cloned parent link name for joint ["
              << clonedJoint << "]" << std::endl;
        continue;
      }
      if (!repaintLinkName(components::ChildLinkName{}, clonedJoint,
              origLinks.second))
      {
        gzerr << "Error updating cloned child link name for joint ["
              << clonedJoint << "]" << std::endl;
        continue;
      }
    }

    return clonedEntity;
  }

  void EntityComponentManager::CopyFrom(
      const EntityComponentManager &_from)
  {
    auto &src = *_from.dataPtr;
    auto &dst = *this->dataPtr;

    // Wipe destination state. Destroy every entity currently in the
    // archetype World, then clear the facade-side bookkeeping.
    for (const auto &[_legacy, core] : dst.legacyToCore)
      dst.world.Destroy(core);
    dst.legacyToCore.clear();
    dst.coreToLegacy.clear();
    dst.shadowComponents.clear();
    dst.newlyCreatedEntities.clear();
    dst.pendingRemovals.clear();
    dst.pinned.clear();
    dst.entityGraph = EntityGraph();

    // Recreate entities in the destination, preserving legacy IDs.
    // The core handles will be different (the new world's
    // EntityIndex allocates fresh), but the legacy↔core map keeps
    // the outside world seeing the same IDs.
    for (const auto &[rawLegacy, _srcCore] : src.legacyToCore)
    {
      Entity e = static_cast<Entity>(rawLegacy);
      gz::sim::ecs::Entity core = dst.world.CreateEmpty();
      dst.legacyToCore.emplace(rawLegacy, core);
      dst.coreToLegacy.emplace(core.Raw(), e);
      dst.entityGraph.AddVertex(std::to_string(e), e, e);
    }

    // Clone shadow components. BaseComponent::Clone() is virtual and
    // does the right thing polymorphically.
    for (const auto &[entity, typeMap] : src.shadowComponents)
    {
      for (const auto &[typeId, comp] : typeMap)
      {
        if (comp)
          dst.shadowComponents[entity][typeId] = comp->Clone();
      }
    }

    // NOTE(0b-copyfrom-archetype-data): Components that live in the
    // archetype World (i.e., added via the template Add<T> path or
    // via World::AddRaw once the Factory bridge lands) are not
    // copied here — we only reach the shadow store. For Phase 0b
    // this is acceptable because every component that lands in a
    // fresh ECM does so via the legacy CreateComponent path, which
    // writes to the shadow store. When the Factory ↔ registry
    // bridge flips the default routing to the archetype World,
    // CopyFrom will need an extra pass that clones columns via
    // ComponentTypeInfo::copy thunks.

    // Copy entity graph edges (parent/child links).
    for (const auto &item : src.entityGraph.Vertices())
    {
      Entity parent = item.second.get().Data();
      for (const auto &childPair :
           src.entityGraph.AdjacentsFrom(parent))
      {
        Entity child = childPair.first;
        dst.entityGraph.AddEdge({parent, child}, true);
      }
    }

    // Copy remaining bookkeeping.
    dst.newlyCreatedEntities = src.newlyCreatedEntities;
    dst.pendingRemovals = src.pendingRemovals;
    dst.pinned = src.pinned;
    dst.nextLegacyId = src.nextLegacyId;
    dst.modifiedComponents = src.modifiedComponents;
    dst.periodicChangedComponents = src.periodicChangedComponents;
    dst.oneTimeChangedComponents = src.oneTimeChangedComponents;
    dst.componentsMarkedAsRemoved = src.componentsMarkedAsRemoved;
    dst.createdCompTypes = src.createdCompTypes;
  }

  void EntityComponentManager::RebuildViews()
  {
    // Archetype queries self-invalidate; this is intentionally a no-op
    // under the archetype backend.
  }

  //----------------------------------------------------------
  // Serialization: State / SetState (SerializedStateMap path).
  //
  // The SerializedStateMap variant is what the GUI, scene broadcaster,
  // and network primary/secondary all use, so getting this working
  // unblocks running gz sim with the GUI connected. Semantics match
  // the legacy implementation (src/EntityComponentManager.cc around
  // line 1490). Reads component values via ComponentImplementation
  // which already checks shadow store first and archetype World
  // second, so both storage paths serialize correctly.
  //
  // The legacy implementation skips unchanged components when _full
  // is false (it tracks one-time and periodic component changes in
  // side tables). The archetype backend's change tracking is
  // stubbed (HasOneTimeComponentChanges/etc. return false), so for
  // now State() emits every component on every call regardless of
  // _full. This is wasteful but correct. Proper change-delta
  // emission is a Phase 0b completion item — marker STUB(0b-delta).
  //----------------------------------------------------------

  void EntityComponentManager::State(
      msgs::SerializedStateMap &_state,
      const std::unordered_set<Entity> &_entities,
      const std::unordered_set<ComponentTypeId> &_types,
      bool _full) const
  {
    // Iterate in sorted (parent-first) order — see
    // AllEntitiesArchetypeFacade for the rationale. _full toggles
    // between full state emission and change-delta: the helper
    // AddEntityToMessage skips unchanged components when
    // _full=false, relying on the periodic/one-time change tables
    // maintained by SetChanged.
    for (Entity entity : this->AllEntitiesArchetypeFacade())
    {
      if (!_entities.empty() && _entities.find(entity) == _entities.end())
        continue;
      this->AddEntityToMessage(_state, entity, _types, _full);
    }
  }

  msgs::SerializedState EntityComponentManager::State(
      const std::unordered_set<Entity> &_entities,
      const std::unordered_set<ComponentTypeId> &_types) const
  {
    // Mirror of the SerializedStateMap variant: iterate in
    // parent-first sorted order (see AllEntitiesArchetypeFacade for
    // the rationale), filter by _entities if non-empty, serialize
    // via AddEntityToMessage.
    msgs::SerializedState stateMsg;
    for (Entity entity : this->AllEntitiesArchetypeFacade())
    {
      if (!_entities.empty() &&
          _entities.find(entity) == _entities.end())
        continue;
      this->AddEntityToMessage(stateMsg, entity, _types);
    }
    return stateMsg;
  }

  msgs::SerializedState EntityComponentManager::ChangedState() const
  {
    // Mirror legacy ordering exactly (src/EntityComponentManager.cc
    // ~ChangedState): newly-created → removed → modified. Tests like
    // EntityComponentManagerFixture.State assert on entities(N) by
    // index, so the order matters.
    msgs::SerializedState stateMsg;
    std::unordered_set<Entity> emitted;

    for (const auto &entity : this->dataPtr->newlyCreatedEntities)
    {
      this->AddEntityToMessage(stateMsg, entity);
      emitted.insert(entity);
    }
    for (const auto &entity : this->dataPtr->pendingRemovals)
    {
      if (!emitted.insert(entity).second) continue;
      this->AddEntityToMessage(stateMsg, entity);
    }
    for (auto entity : this->dataPtr->modifiedComponents)
    {
      if (!emitted.insert(entity).second) continue;
      this->AddEntityToMessage(stateMsg, entity);
    }
    return stateMsg;
  }

  void EntityComponentManager::ChangedState(
      msgs::SerializedStateMap &_state) const
  {
    // Same legacy ordering for the SerializedStateMap variant.
    std::unordered_set<Entity> emitted;
    for (const auto &entity : this->dataPtr->newlyCreatedEntities)
    {
      this->AddEntityToMessage(_state, entity, {}, /*_full=*/false);
      emitted.insert(entity);
    }
    for (const auto &entity : this->dataPtr->pendingRemovals)
    {
      if (!emitted.insert(entity).second) continue;
      this->AddEntityToMessage(_state, entity, {}, /*_full=*/false);
    }
    for (auto entity : this->dataPtr->modifiedComponents)
    {
      if (!emitted.insert(entity).second) continue;
      this->AddEntityToMessage(_state, entity, {}, /*_full=*/false);
    }
  }

  void EntityComponentManager::SetState(
      const msgs::SerializedStateMap &_stateMsg)
  {
    // Mirror legacy semantics (src/EntityComponentManager.cc ~1911).
    for (const auto &iter : _stateMsg.entities())
    {
      const auto &entityMsg = iter.second;
      Entity entity{entityMsg.id()};

      if (entityMsg.remove())
      {
        this->RequestRemoveEntity(entity);
        continue;
      }

      // Create entity if it doesn't exist. The legacy version calls
      // a private CreateEntityImplementation(Entity) that preserves
      // the ID from the message; we do the equivalent here, keeping
      // the id in legacyToCore and the entity graph.
      if (!this->HasEntity(entity))
      {
        gz::sim::ecs::Entity core = this->dataPtr->world.CreateEmpty();
        this->dataPtr->legacyToCore.emplace(
            static_cast<uint64_t>(entity), core);
        this->dataPtr->coreToLegacy.emplace(core.Raw(), entity);
        this->dataPtr->entityGraph.AddVertex(
            std::to_string(entity), entity, entity);
        this->dataPtr->newlyCreatedEntities.insert(entity);
        // Track future IDs so auto-allocation doesn't collide with
        // peer-assigned ones.
        if (entity >= this->dataPtr->nextLegacyId)
          this->dataPtr->nextLegacyId = entity + 1;
      }

      for (const auto &compIter : entityMsg.components())
      {
        const auto &compMsg = compIter.second;
        uint64_t type = compMsg.type();

        // Components that aren't registered locally can't be
        // deserialized — skip with a one-shot warning (legacy does
        // the same).
        if (!components::Factory::Instance()->HasType(type))
        {
          static std::unordered_set<unsigned int> printedComps;
          if (printedComps.find(type) == printedComps.end())
          {
            printedComps.insert(type);
            gzwarn << "Component type [" << type << "] not registered"
                   << " in this process; can't deserialize."
                   << std::endl;
          }
          continue;
        }

        if (compMsg.remove())
        {
          this->RemoveComponent(entity, compIter.first);
          continue;
        }

        components::BaseComponent *comp =
            this->ComponentImplementation(entity, compIter.first);

        if (nullptr == comp)
        {
          // Fresh component — materialize via the Factory, deserialize,
          // then hand to CreateComponentImplementation.
          auto newComp = components::Factory::Instance()->New(type);
          if (!newComp)
          {
            gzerr << "Failed to create component of type [" << type
                  << "]" << std::endl;
            continue;
          }
          std::istringstream istr(compMsg.component());
          newComp->Deserialize(istr);

          auto updateData = this->CreateComponentImplementation(
              entity, newComp->TypeId(), newComp.get());
          if (updateData)
          {
            comp = this->ComponentImplementation(entity, compIter.first);
          }
        }

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

  void EntityComponentManager::SetState(
      const msgs::SerializedState &_stateMsg)
  {
    // Plain (non-map) variant. Same semantics as the SerializedStateMap
    // version, just iterating the repeated-field API.
    for (int e = 0; e < _stateMsg.entities_size(); ++e)
    {
      const auto &entityMsg = _stateMsg.entities(e);
      Entity entity{entityMsg.id()};

      if (entityMsg.remove())
      {
        this->RequestRemoveEntity(entity);
        continue;
      }

      if (!this->HasEntity(entity))
      {
        gz::sim::ecs::Entity core = this->dataPtr->world.CreateEmpty();
        this->dataPtr->legacyToCore.emplace(
            static_cast<uint64_t>(entity), core);
        this->dataPtr->coreToLegacy.emplace(core.Raw(), entity);
        this->dataPtr->entityGraph.AddVertex(
            std::to_string(entity), entity, entity);
        this->dataPtr->newlyCreatedEntities.insert(entity);
        if (entity >= this->dataPtr->nextLegacyId)
          this->dataPtr->nextLegacyId = entity + 1;
      }

      for (int c = 0; c < entityMsg.components_size(); ++c)
      {
        const auto &compMsg = entityMsg.components(c);

        // Skip if component not set (matches legacy behavior).
        if (compMsg.component().empty()) continue;

        auto type = compMsg.type();

        if (!components::Factory::Instance()->HasType(type))
        {
          static std::unordered_set<unsigned int> printedComps;
          if (printedComps.find(type) == printedComps.end())
          {
            printedComps.insert(type);
            gzwarn << "Component type [" << type << "] not registered"
                   << " in this process; can't deserialize."
                   << std::endl;
          }
          continue;
        }

        if (compMsg.remove())
        {
          this->RemoveComponent(entity, type);
          continue;
        }

        components::BaseComponent *comp =
            this->ComponentImplementation(entity, type);

        if (nullptr == comp)
        {
          auto newComp = components::Factory::Instance()->New(type);
          if (!newComp)
          {
            gzerr << "Failed to create component type [" << type
                  << "]" << std::endl;
            continue;
          }
          std::istringstream istr(compMsg.component());
          newComp->Deserialize(istr);

          auto updateData = this->CreateComponentImplementation(
              entity, newComp->TypeId(), newComp.get());
          if (updateData)
            comp = this->ComponentImplementation(entity, type);
        }

        if (comp)
        {
          std::istringstream istr(compMsg.component());
          comp->Deserialize(istr);
        }
      }
    }
  }

  void EntityComponentManager::PeriodicStateFromCache(
      msgs::SerializedStateMap &_state,
      const std::unordered_map<ComponentTypeId,
                               std::unordered_set<Entity>> &_cache) const
  {
    // Scene broadcaster's per-step periodic emission: the cache
    // identifies which (entity, typeId) pairs are currently tracked
    // as periodic-change candidates. Serialize those components into
    // the state message. The legacy implementation at
    // src/EntityComponentManager.cc ~1771 is the reference.
    for (const auto &[typeId, entities] : _cache)
    {
      for (const auto &entity : entities)
      {
        auto entIter = _state.mutable_entities()->find(entity);
        if (entIter == _state.mutable_entities()->end())
        {
          msgs::SerializedEntityMap ent;
          ent.set_id(entity);
          (*_state.mutable_entities())[static_cast<uint64_t>(entity)] = ent;
          entIter = _state.mutable_entities()->find(entity);
        }

        // If the component is already in the message, skip — State()
        // likely put it there already and periodic re-emission would
        // duplicate. Matches legacy (src/EntityComponentManager.cc ~1791).
        auto compIter = entIter->second.mutable_components()->find(typeId);
        if (compIter != entIter->second.mutable_components()->end())
          continue;

        const components::BaseComponent *compBase =
            this->ComponentImplementation(entity, typeId);
        if (!compBase) continue;

        msgs::SerializedComponent cmp;
        cmp.set_type(compBase->TypeId());
        std::ostringstream ostr;
        compBase->Serialize(ostr);
        cmp.set_component(ostr.str());
        (*(entIter->second.mutable_components()))[
            static_cast<int64_t>(typeId)] = cmp;
      }
    }
  }

  std::unordered_set<ComponentTypeId>
  EntityComponentManager::ComponentTypesWithPeriodicChanges() const
  {
    std::unordered_set<ComponentTypeId> out;
    for (const auto &[type, ents] : this->dataPtr->periodicChangedComponents)
      if (!ents.empty()) out.insert(type);
    return out;
  }

  void EntityComponentManager::UpdatePeriodicChangeCache(
      std::unordered_map<ComponentTypeId,
                         std::unordered_set<Entity>> &_changes) const
  {
    // Mirror legacy (src/EntityComponentManager.cc UpdatePeriodicChangeCache):
    //   1. Merge current periodic changes into the cache.
    //   2. Drop (entity, type) pairs whose component was just removed.
    //   3. Drop (entity, *) pairs for entities pending removal.
    for (const auto &[type, ents] : this->dataPtr->periodicChangedComponents)
      _changes[type].insert(ents.begin(), ents.end());

    for (const auto &[entity, comps] :
        this->dataPtr->componentsMarkedAsRemoved)
    {
      for (auto comp : comps)
      {
        auto it = _changes.find(comp);
        if (it != _changes.end())
          it->second.erase(entity);
      }
    }

    for (auto entity : this->dataPtr->pendingRemovals)
    {
      for (auto &kv : _changes)
        kv.second.erase(entity);
    }
  }

  void EntityComponentManager::SetChanged(
      const Entity _entity, const ComponentTypeId _type,
      sim::ComponentState _c)
  {
    // Mirror legacy SetChanged at src/EntityComponentManager.cc ~2040:
    // maintain two side tables (periodic vs one-time) that ChangedState
    // / PeriodicStateFromCache / State(full=false) can query. NoChange
    // drops from both tables.
    if (!this->HasEntity(_entity)) return;
    if (!this->EntityHasComponentType(_entity, _type)) return;

    if (_c == sim::ComponentState::PeriodicChange)
    {
      this->dataPtr->periodicChangedComponents[_type].insert(_entity);
      auto it = this->dataPtr->oneTimeChangedComponents.find(_type);
      if (it != this->dataPtr->oneTimeChangedComponents.end())
        it->second.erase(_entity);
      this->dataPtr->AddModifiedComponent(_entity);
    }
    else if (_c == sim::ComponentState::OneTimeChange)
    {
      auto it = this->dataPtr->periodicChangedComponents.find(_type);
      if (it != this->dataPtr->periodicChangedComponents.end())
        it->second.erase(_entity);
      this->dataPtr->oneTimeChangedComponents[_type].insert(_entity);
      this->dataPtr->AddModifiedComponent(_entity);
    }
    else
    {
      // NoChange.
      auto pit = this->dataPtr->periodicChangedComponents.find(_type);
      if (pit != this->dataPtr->periodicChangedComponents.end())
        pit->second.erase(_entity);
      auto oit = this->dataPtr->oneTimeChangedComponents.find(_type);
      if (oit != this->dataPtr->oneTimeChangedComponents.end())
        oit->second.erase(_entity);
    }
  }

  sim::ComponentState EntityComponentManager::ComponentState(
      const Entity _entity, const ComponentTypeId _typeId) const
  {
    auto pit = this->dataPtr->periodicChangedComponents.find(_typeId);
    if (pit != this->dataPtr->periodicChangedComponents.end() &&
        pit->second.count(_entity))
      return sim::ComponentState::PeriodicChange;
    auto oit = this->dataPtr->oneTimeChangedComponents.find(_typeId);
    if (oit != this->dataPtr->oneTimeChangedComponents.end() &&
        oit->second.count(_entity))
      return sim::ComponentState::OneTimeChange;
    return sim::ComponentState::NoChange;
  }

  void EntityComponentManager::SetEntityCreateOffset(uint64_t _offset)
  {
    // Legacy semantics: next CreateEntity returns (_offset + 1).
    // Legacy uses pre-increment on entityCount; we use post-increment
    // on nextLegacyId, so add 1 here to keep IDs aligned with the
    // legacy value the test (and downstream code) expects.
    this->dataPtr->nextLegacyId = _offset + 1;
  }

  void EntityComponentManager::ResetTo(const EntityComponentManager &_other)
  {
    // Same algorithm as legacy (src/EntityComponentManager.cc
    // EntityComponentManager::ResetTo): bring `_other`'s state into
    // `this`, but preserve `this`'s currently-live entities by
    // routing them through a tmp ECM and the diff/apply machinery.
    auto ecmDiff = this->ComputeEntityDiff(_other);
    EntityComponentManager tmpCopy;
    tmpCopy.CopyFrom(_other);
    tmpCopy.ApplyEntityDiff(*this, ecmDiff);
    this->CopyFrom(tmpCopy);
  }

  std::optional<Entity> EntityComponentManager::EntityByName(
      const std::string &_name) const
  {
    // Linear scan — every entity that has a Name component gets
    // compared. For large worlds this is O(N); a per-ECM name index
    // would be O(1) but requires maintaining the index on every
    // Name creation/removal. Given in-tree callers are setup-only
    // (e.g., SdfEntityCreator disambiguating model ownership), the
    // linear scan is acceptable for Phase 0b.
    for (Entity e : this->dataPtr->legacyToCore.empty()
        ? std::vector<Entity>{}
        : this->AllEntitiesArchetypeFacade())
    {
      auto *n = this->Component<components::Name>(e);
      if (n && n->Data() == _name) return e;
    }
    return std::nullopt;
  }

  EntityComponentManagerDiff EntityComponentManager::ComputeEntityDiff(
      const EntityComponentManager &_other) const
  {
    // Mirror legacy ComputeEntityDiff (src/EntityComponentManager.cc):
    // - entities present in `_other` but not in `this` → "added"
    // - entities present in `this` but not in `_other` → "removed"
    EntityComponentManagerDiff diff;
    for (const auto &[rawLegacy, _core] : _other.dataPtr->legacyToCore)
    {
      Entity e = static_cast<Entity>(rawLegacy);
      if (!this->dataPtr->legacyToCore.count(rawLegacy))
        diff.InsertAddedEntity(e);
    }
    for (const auto &[rawLegacy, _core] : this->dataPtr->legacyToCore)
    {
      Entity e = static_cast<Entity>(rawLegacy);
      if (!_other.dataPtr->legacyToCore.count(rawLegacy))
        diff.InsertRemovedEntity(e);
    }
    return diff;
  }

  void EntityComponentManager::ApplyEntityDiff(
      const EntityComponentManager &_other,
      const EntityComponentManagerDiff &_diff)
  {
    // Mirror legacy ApplyEntityDiff. For each entity flagged as
    // "added" (in `_other` but not in `this`), recreate it and copy
    // its components. For each "removed", create it and immediately
    // mark it for removal so EachRemoved/Each pickups behave
    // correctly downstream.
    auto copyComponents = [&](Entity _e)
    {
      for (const auto compTypeId : _other.ComponentTypes(_e))
      {
        const components::BaseComponent *data =
            _other.ComponentImplementation(_e, compTypeId);
        if (!data) continue;
        auto cloned = data->Clone();
        if (!cloned) continue;
        this->CreateComponentImplementation(_e, compTypeId, cloned.get());
      }
    };

    auto materializeEntity = [&](Entity _e)
    {
      if (this->HasEntity(_e)) return;
      gz::sim::ecs::Entity core = this->dataPtr->world.CreateEmpty();
      this->dataPtr->legacyToCore.emplace(
          static_cast<uint64_t>(_e), core);
      this->dataPtr->coreToLegacy.emplace(core.Raw(), _e);
      this->dataPtr->entityGraph.AddVertex(
          std::to_string(_e), _e, _e);
      this->dataPtr->newlyCreatedEntities.insert(_e);
      if (_e >= this->dataPtr->nextLegacyId)
        this->dataPtr->nextLegacyId = _e + 1;
    };

    for (auto entity : _diff.AddedEntities())
    {
      materializeEntity(entity);
      copyComponents(entity);
      auto p = _other.ParentEntity(entity);
      if (p != kNullEntity) this->SetParentEntity(entity, p);
    }

    for (const auto &entity : _diff.RemovedEntities())
    {
      if (!this->HasEntity(entity))
      {
        materializeEntity(entity);
        // ApplyEntityDiff treats these as already-removed, not new.
        this->dataPtr->newlyCreatedEntities.erase(entity);
        copyComponents(entity);
        auto p = _other.ParentEntity(entity);
        if (p != kNullEntity) this->SetParentEntity(entity, p);
      }
      this->RequestRemoveEntity(entity, /*_recursive=*/false);
    }
  }

  bool EntityComponentManager::IsNewEntity(const Entity _entity) const
  {
    return this->dataPtr->newlyCreatedEntities.count(_entity) > 0;
  }

  bool EntityComponentManager::IsMarkedForRemoval(const Entity _entity) const
  {
    return std::find(this->dataPtr->pendingRemovals.begin(),
                     this->dataPtr->pendingRemovals.end(),
                     _entity) != this->dataPtr->pendingRemovals.end();
  }

  void EntityComponentManager::AddEntityToMessage(
      msgs::SerializedState &_msg,
      Entity _entity,
      const std::unordered_set<ComponentTypeId> &_types) const
  {
    auto entityMsg = _msg.add_entities();
    entityMsg->set_id(_entity);
    if (!this->HasEntity(_entity)) return;

    // Mark for removal if pending.
    if (std::find(this->dataPtr->pendingRemovals.begin(),
                  this->dataPtr->pendingRemovals.end(), _entity) !=
        this->dataPtr->pendingRemovals.end())
    {
      entityMsg->set_remove(true);
    }

    auto types = _types;
    if (types.empty())
    {
      types = this->ComponentTypes(_entity);
      // Pattern B: include components that have been marked-removed
      // this step so the receiver can clear its mirror.
      auto rit = this->dataPtr->componentsMarkedAsRemoved.find(_entity);
      if (rit != this->dataPtr->componentsMarkedAsRemoved.end())
        types.insert(rit->second.begin(), rit->second.end());
    }

    for (const ComponentTypeId type : types)
    {
      bool isRemoved = this->dataPtr->IsMarkedRemoved(_entity, type);
      auto compMsg = entityMsg->add_components();
      compMsg->set_type(type);
      if (isRemoved)
      {
        compMsg->set_remove(true);
        continue;
      }

      const components::BaseComponent *compBase =
          this->ComponentImplementation(_entity, type);
      if (!compBase)
      {
        // Component disappeared between ComponentTypes() and now —
        // remove the trailing entry rather than emit garbage.
        entityMsg->mutable_components()->RemoveLast();
        continue;
      }

      std::ostringstream ostr;
      compBase->Serialize(ostr);
      compMsg->set_component(ostr.str());
    }
  }

  void EntityComponentManager::AddEntityToMessage(
      msgs::SerializedStateMap &_msg,
      Entity _entity,
      const std::unordered_set<ComponentTypeId> &_types,
      bool _full) const
  {
    if (!this->HasEntity(_entity)) return;

    auto types = _types;
    if (types.empty())
    {
      types = this->ComponentTypes(_entity);
      // Pattern B: include components that have been marked-removed
      // this step. The legacy state message carries them with
      // remove=true so the receiver can clear its mirror.
      auto rit = this->dataPtr->componentsMarkedAsRemoved.find(_entity);
      if (rit != this->dataPtr->componentsMarkedAsRemoved.end())
        types.insert(rit->second.begin(), rit->second.end());
    }
    if (types.empty() &&
        std::find(this->dataPtr->pendingRemovals.begin(),
                  this->dataPtr->pendingRemovals.end(), _entity) ==
            this->dataPtr->pendingRemovals.end())
      return;

    // Add the entity sub-message if not already present.
    auto entIter = _msg.mutable_entities()->find(_entity);
    auto ensureEntity = [&]() {
      if (entIter == _msg.mutable_entities()->end())
      {
        msgs::SerializedEntityMap ent;
        ent.set_id(_entity);
        (*_msg.mutable_entities())[static_cast<uint64_t>(_entity)] = ent;
        entIter = _msg.mutable_entities()->find(_entity);
      }
    };

    // Mark removal, matching legacy semantics. Always emitted (even
    // when _full=false) because removals are one-time events.
    bool pending = std::find(this->dataPtr->pendingRemovals.begin(),
                             this->dataPtr->pendingRemovals.end(),
                             _entity) !=
                   this->dataPtr->pendingRemovals.end();
    if (pending)
    {
      ensureEntity();
      entIter->second.set_remove(true);
    }

    for (const ComponentTypeId type : types)
    {
      // Pattern B: marked-as-removed components are emitted as
      // remove=true. Always include them regardless of _full so
      // receivers see the removal.
      bool isRemoved = this->dataPtr->IsMarkedRemoved(_entity, type);

      // Delta mode: skip components that aren't marked as changed
      // (and aren't removed). CreateComponent auto-marks the new
      // (entity,type) pair in oneTimeChangedComponents, so brand-new
      // components on brand-new entities still flow through.
      if (!_full && !isRemoved)
      {
        bool changed = false;
        auto oit = this->dataPtr->oneTimeChangedComponents.find(type);
        if (oit != this->dataPtr->oneTimeChangedComponents.end() &&
            oit->second.count(_entity))
          changed = true;
        if (!changed)
        {
          auto pit = this->dataPtr->periodicChangedComponents.find(type);
          if (pit != this->dataPtr->periodicChangedComponents.end() &&
              pit->second.count(_entity))
            changed = true;
        }
        if (!changed) continue;
      }

      ensureEntity();
      auto compIter = entIter->second.mutable_components()->find(type);
      if (compIter == entIter->second.mutable_components()->end())
      {
        msgs::SerializedComponent cmp;
        cmp.set_type(type);
        (*(entIter->second.mutable_components()))[
            static_cast<int64_t>(type)] = cmp;
        compIter = entIter->second.mutable_components()->find(type);
      }

      if (isRemoved)
      {
        compIter->second.set_remove(true);
        continue;
      }

      const components::BaseComponent *compBase =
          this->ComponentImplementation(_entity, type);
      if (!compBase) continue;

      std::ostringstream ostr;
      compBase->Serialize(ostr);
      compIter->second.set_component(ostr.str());
    }
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
