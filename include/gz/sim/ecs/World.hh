/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_WORLD_HH_
#define GZ_SIM_ECS_WORLD_HH_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "gz/sim/ecs/CommandBuffer.hh"
#include "gz/sim/ecs/ComponentTypeRegistry.hh"
#include "gz/sim/ecs/Entity.hh"
#include "gz/sim/ecs/Query.hh"

namespace gz::common { class WorkerPool; }

namespace gz::sim::ecs
{
  // Forward declarations (internal types kept out of the public header).
  class Archetype;
  class ArchetypeGraph;
  class EntityIndex;

  /// \brief The World owns all entities, archetypes, and chunks. It is the
  /// hub of the Phase 0a archetype ECS.
  class World
  {
    public: World();
    public: explicit World(unsigned int _num_threads);
    public: ~World();

    public: World(const World &) = delete;
    public: World &operator=(const World &) = delete;

    // -------- Entity lifecycle --------

    /// \brief Create an entity with the given component pack. Immediate
    /// outside a phase; deferred inside a phase.
    public: template <class... Cs>
    Entity Create(Cs &&... _cs)
    {
      if (this->in_phase_)
      {
        return this->DeferredCreate(std::forward<Cs>(_cs)...);
      }
      return this->ImmediateCreate(std::forward<Cs>(_cs)...);
    }

    /// \brief Create an entity with no components.
    public: Entity CreateEmpty();

    /// \brief Destroy an entity.
    public: void Destroy(Entity _e);

    /// \brief Add (or overwrite) a component on an entity.
    public: template <class C>
    void Add(Entity _e, C &&_c)
    {
      using T = std::decay_t<C>;
      auto *info = ComponentTypeRegistry::Instance().Register<T>();
      if (this->in_phase_)
      {
        this->DeferredAdd(_e, info, &_c, /*is_rvalue*/
            std::is_rvalue_reference_v<C &&>);
        return;
      }
      this->ImmediateAdd(_e, info, std::addressof(_c),
          std::is_rvalue_reference_v<C &&>);
    }

    /// \brief Remove a component from an entity.
    public: template <class C>
    void Remove(Entity _e)
    {
      auto tid = ComponentTypeRegistry::Instance().TypeIdOf<C>();
      if (this->in_phase_)
      {
        this->DeferredRemove(_e, tid);
        return;
      }
      this->ImmediateRemove(_e, tid);
    }

    // -------- Queries --------

    public: bool IsAlive(Entity _e) const;

    public: bool Has(Entity _e, ComponentTypeId _id) const;
    public: template <class C> bool Has(Entity _e) const
    {
      return this->Has(_e, ComponentTypeRegistry::Instance().TypeIdOf<C>());
    }

    public: template <class C>
    const C *Component(Entity _e) const
    {
      return static_cast<const C *>(this->ComponentRaw(_e,
          ComponentTypeRegistry::Instance().TypeIdOf<C>()));
    }

    public: template <class C>
    C *Component(Entity _e)
    {
      return static_cast<C *>(this->ComponentRawMut(_e,
          ComponentTypeRegistry::Instance().TypeIdOf<C>()));
    }

    /// \brief Mark a component dirty for change tracking. Prefer calling
    /// Modify<T> on write rather than touching Component<T> — this is the
    /// explicit, preferred path for change tracking.
    public: template <class C>
    C *Modify(Entity _e)
    {
      auto tid = ComponentTypeRegistry::Instance().TypeIdOf<C>();
      return static_cast<C *>(this->ComponentRawMutAndDirty(_e, tid));
    }

    // -------- Iteration --------

    /// \brief Walk every entity that has all of Cs.... Caller-provided
    /// functor signature: fn(Entity, Cs&...). const Cs are honored.
    public: template <class... Cs, class Fn>
    void Each(Fn &&_fn);

    /// \brief Parallel sweep across chunks using a simple std::thread pool
    /// owned by the World.
    public: template <class... Cs, class Fn>
    void EachParallel(Fn &&_fn);

    /// \brief Walk entities whose archetype is newly populated with a row
    /// (created this step, either via Create<T> or via an Add that
    /// transitioned the entity into an archetype containing T). Cleared by
    /// ClearChangeBits.
    public: template <class... Cs, class Fn>
    void EachNew(Fn &&_fn);

    /// \brief Walk entities for which at least one of Cs was modified
    /// (Modify<T> called, or the Modify-guard path) since the last
    /// ClearChangeBits.
    public: template <class... Cs, class Fn>
    void EachChanged(Fn &&_fn);

    /// \brief Per-step drain of removal records (rows queued during the
    /// step, captured before actual deletion). Cleared by ClearChangeBits.
    public: template <class... Cs, class Fn>
    void EachRemoved(Fn &&_fn);

    // -------- Phase semantics --------

    /// \brief Begin a phase — subsequent mutations are deferred.
    public: void BeginPhase();

    /// \brief Apply all buffered mutations. Bumps the chunk generation.
    public: void Commit();

    /// \brief Clear all change-tracking state (dirty bits, newly-created /
    /// removed vectors). Typically called after the final Commit of a step.
    public: void ClearChangeBits();

    /// \brief Bump-only counter incremented by every Commit(). Used by the
    /// debug pointer-validity checks.
    public: uint64_t ChunkGeneration() const { return this->chunk_generation_; }

    // -------- Access for Query / testing --------

    public: const ArchetypeGraph &Graph() const { return *this->graph_; }
    public: ArchetypeGraph       &Graph()       { return *this->graph_; }

    public: size_t NumEntities() const;
    public: size_t NumArchetypes() const;

    public: unsigned int NumThreads() const { return this->num_threads_; }

    // -------- Internal (used by templates) --------

    // Phase 0b: made public so the archetype-backed facade in
    // src/EntityComponentManagerArchetype.cc can reach component data
    // by runtime typeId without going through a template. Previously
    // private; the template Component<T>(e) still routes through these.
    public: const void *ComponentRaw(Entity _e, ComponentTypeId _id) const;
    public: void       *ComponentRawMut(Entity _e, ComponentTypeId _id);

    /// \brief Remove a component by runtime typeId. Complements the
    /// template `Remove<T>()`. Phase 0b facade uses this for the
    /// `EntityComponentManager::RemoveComponent(entity, typeId)`
    /// public method.
    public: void RemoveRaw(Entity _e, ComponentTypeId _id);

    /// \brief Add a component by runtime typeId with raw source
    /// bytes. Complements the template `Add<T>()`. Requires that the
    /// type has already been registered with the
    /// ComponentTypeRegistry — caller is responsible for that.
    ///
    /// Phase 0b facade uses this for `CreateComponentImplementation`
    /// when the typeId is registered. When it isn't (the common
    /// initial-ship case), the facade falls back to a shadow BaseComponent
    /// map; see src/EntityComponentManagerArchetype.cc for the
    /// NOTE(0b-shadow-store) commentary.
    public: void AddRaw(Entity _e, ComponentTypeId _id, const void *_src);
    private: void       *ComponentRawMutAndDirty(Entity _e, ComponentTypeId _id);

    private: Entity ImmediateCreateFromBlob(
        const std::vector<std::pair<const ComponentTypeInfo *, const void *>>
            &_pack);

    private: template <class... Cs>
    Entity ImmediateCreate(Cs &&... _cs)
    {
      auto &reg = ComponentTypeRegistry::Instance();
      std::vector<std::pair<const ComponentTypeInfo *, const void *>> pack;
      pack.reserve(sizeof...(Cs));
      (pack.emplace_back(reg.Register<std::decay_t<Cs>>(),
                         static_cast<const void *>(std::addressof(_cs))), ...);
      return this->ImmediateCreateFromBlob(pack);
    }

    private: template <class... Cs>
    Entity DeferredCreate(Cs &&... _cs);

    private: void ImmediateAdd(Entity _e, const ComponentTypeInfo *_info,
                               void *_src, bool _src_is_rvalue);
    private: void ImmediateRemove(Entity _e, ComponentTypeId _id);

    private: void DeferredAdd(Entity _e, const ComponentTypeInfo *_info,
                              void *_src, bool _src_is_rvalue);
    private: void DeferredRemove(Entity _e, ComponentTypeId _id);

    // Thread-local command-buffer access.
    private: CommandBuffer &LocalBuffer();

    // Reserve N consecutive command-buffer slots and return the
    // first slot's index. Used by EachParallel to pre-register
    // worker slots in deterministic order before dispatching work
    // — without this, slot assignment via LocalBuffer() depends on
    // which worker thread happens to call first, which makes
    // Commit's merge order non-deterministic. This API is internal.
    public: size_t ReserveBufferSlots(size_t _n);

    // Direct buffer access by pre-reserved slot index. Bypasses
    // the thread-id lookup that LocalBuffer() performs. EachParallel
    // workers use this to write into their pre-assigned slot.
    public: CommandBuffer &BufferAtSlot(size_t _slot);

    // RAII guard that pins LocalBuffer() to a specific slot for
    // the calling thread's lifetime of the guard. EachParallel
    // wraps each worker's user-body call so deferred mutations
    // route through the worker's pre-assigned slot. Implementation
    // in World.cc.
    public: struct SlotOverrideGuard
    {
      size_t prev;
      explicit SlotOverrideGuard(size_t _slot);
      ~SlotOverrideGuard();
      SlotOverrideGuard(const SlotOverrideGuard &) = delete;
      SlotOverrideGuard &operator=(const SlotOverrideGuard &) = delete;
    };

    private: friend class Query;
    private: friend class QueryTestAccess;

    private: std::unique_ptr<ArchetypeGraph>       graph_;
    private: std::unique_ptr<EntityIndex>          entity_index_;
    private: std::unique_ptr<gz::common::WorkerPool> worker_pool_;

    // Exposed so template EachParallel (in WorldImpl.hh) can dispatch work.
    public: gz::common::WorkerPool *WorkerPoolInternal()
    {
      return this->worker_pool_.get();
    }

    // Main-thread command buffer (index 0); extra slots for worker threads.
    private: std::vector<std::unique_ptr<CommandBuffer>> command_buffers_;
    private: std::mutex command_buffers_mutex_;

    // Per-thread-id -> buffer index.
    private: struct ThreadSlot
    {
      std::thread::id id;
      size_t          index;
    };
    private: std::vector<ThreadSlot> thread_slots_;

    private: bool        in_phase_{false};
    private: uint64_t    chunk_generation_{0};
    private: unsigned int num_threads_{0};

    // Removal records, captured before deletion, drained by EachRemoved.
    public: struct RemovedRecord
    {
      Entity                       entity;
      std::vector<ComponentTypeId> types;
    };
    private: std::vector<RemovedRecord> removed_records_;

    public: const std::vector<RemovedRecord> &RemovedRecords() const
    { return this->removed_records_; }
  };
}

#include "gz/sim/ecs/detail/WorldImpl.hh"

#endif
