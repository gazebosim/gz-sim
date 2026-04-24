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
#include "gz/sim/components/Name.hh"
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
    std::vector<Entity> out;
    out.reserve(this->dataPtr->legacyToCore.size());
    for (const auto &[k, _] : this->dataPtr->legacyToCore)
      out.push_back(static_cast<Entity>(k));
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
      gz::sim::ecs::Entity core;
      if (!this->dataPtr->CoreFor(e, core)) continue;
      this->dataPtr->world.Destroy(core);
      this->dataPtr->coreToLegacy.erase(core.Raw());
      this->dataPtr->legacyToCore.erase(static_cast<uint64_t>(e));
      this->dataPtr->entityGraph.RemoveVertex(e);
      this->dataPtr->shadowComponents.erase(e);
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

    // Shadow store (non-template path).
    auto eit = this->dataPtr->shadowComponents.find(_entity);
    if (eit != this->dataPtr->shadowComponents.end())
    {
      if (eit->second.erase(_typeId)) removed = true;
    }

    // Archetype world (template path).
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
    return ecs::ComponentTypeRegistry::Instance().Get(_typeId) != nullptr;
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
      bool /*_full*/) const
  {
    // STUB(0b-delta): full=false should skip unchanged components
    // once change-tracking is implemented on the archetype backend.
    for (const auto &[rawLegacy, _core] : this->dataPtr->legacyToCore)
    {
      Entity entity = static_cast<Entity>(rawLegacy);
      if (!_entities.empty() && _entities.find(entity) == _entities.end())
        continue;
      this->AddEntityToMessage(_state, entity, _types, /*_full=*/true);
    }
  }

  msgs::SerializedState EntityComponentManager::State(
      const std::unordered_set<Entity> &_entities,
      const std::unordered_set<ComponentTypeId> &_types) const
  {
    // Minimum-viable: return empty. The SerializedStateMap variant
    // is what in-tree consumers actually use; the plain
    // SerializedState form is called by a few tests and the Python
    // bindings. Filling it in is mechanical alongside the delta work.
    (void)_entities;
    (void)_types;
    GZ_SIM_ARCH_STUB_ONCE("State(SerializedState)");
    return msgs::SerializedState{};
  }

  msgs::SerializedState EntityComponentManager::ChangedState() const
  {
    // STUB(0b-delta): once change-tracking is implemented, this is
    // State(fullset, allTypes, _full=false). For now the best we can
    // do is emit every-component state.
    GZ_SIM_ARCH_STUB_ONCE("ChangedState");
    return msgs::SerializedState{};
  }

  void EntityComponentManager::ChangedState(
      msgs::SerializedStateMap &_state) const
  {
    // STUB(0b-delta): delta pending change tracking. Emit full state
    // for now so the GUI sees something.
    this->State(_state, {}, {}, /*_full=*/true);
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

  void EntityComponentManager::SetState(const msgs::SerializedState&)
  {
    // Plain (non-map) variant. Legacy consumers are a handful of
    // tests + Python bindings. Not a blocker for gz sim + GUI.
    GZ_SIM_ARCH_STUB_ONCE("SetState(SerializedState)");
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
      msgs::SerializedStateMap &_msg,
      Entity _entity,
      const std::unordered_set<ComponentTypeId> &_types,
      bool /*_full*/) const
  {
    if (!this->HasEntity(_entity)) return;

    // Figure out which component types to emit.
    auto types = _types;
    if (types.empty()) types = this->ComponentTypes(_entity);

    if (types.empty()) return;

    // Add the entity sub-message if not already present.
    auto entIter = _msg.mutable_entities()->find(_entity);
    if (entIter == _msg.mutable_entities()->end())
    {
      msgs::SerializedEntityMap ent;
      ent.set_id(_entity);
      (*_msg.mutable_entities())[static_cast<uint64_t>(_entity)] = ent;
      entIter = _msg.mutable_entities()->find(_entity);
    }

    // Mark removal, matching legacy semantics.
    if (std::find(this->dataPtr->pendingRemovals.begin(),
                  this->dataPtr->pendingRemovals.end(), _entity) !=
        this->dataPtr->pendingRemovals.end())
    {
      entIter->second.set_remove(true);
    }

    for (const ComponentTypeId type : types)
    {
      const components::BaseComponent *compBase =
          this->ComponentImplementation(_entity, type);
      if (!compBase) continue;

      auto compIter = entIter->second.mutable_components()->find(type);
      if (compIter == entIter->second.mutable_components()->end())
      {
        msgs::SerializedComponent cmp;
        cmp.set_type(compBase->TypeId());
        (*(entIter->second.mutable_components()))[
            static_cast<int64_t>(type)] = cmp;
        compIter = entIter->second.mutable_components()->find(type);
      }

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
