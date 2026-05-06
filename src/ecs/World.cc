/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// World — the public entrypoint to the archetype ECS core.
//
// A `World` owns:
//   * The `ArchetypeGraph` (the registry of archetypes + their
//     transition edges) and the `EntityIndex` (entity → location
//     map).
//   * One `CommandBuffer` per worker thread, vended by `LocalBuffer`,
//     used to defer mutations issued during a phase.
//   * A `WorkerPool` for `EachParallel`.
//   * Per-step "removed records" — the list of entities destroyed
//     this step, used by `EachRemoved` consumers (logging,
//     scene_broadcaster).
//
// Every public mutation method (`CreateEmpty`, `Destroy`, `AddRaw`,
// `RemoveRaw`) checks `in_phase_` and either applies immediately or
// queues into the local thread's command buffer. `BeginPhase()`
// arms the deferred path; `Commit()` drains every buffer in
// deterministic order and applies the queued ops, then bumps the
// chunk-generation counter so any cached `Component<T>(e)` pointer
// is observably invalidated.
//
// Outside a phase (the default state), the immediate path runs
// directly. This is what gives the legacy ECM facade in
// `src/EntityComponentManagerArchetype.cc` its "immediate-mutation
// mode" semantics today.
#include "gz/sim/ecs/World.hh"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>

#include <gz/common/WorkerPool.hh>

#include "gz/sim/ecs/detail/Archetype.hh"
#include "gz/sim/ecs/detail/ArchetypeGraph.hh"
#include "gz/sim/ecs/detail/Chunk.hh"
#include "gz/sim/ecs/detail/EntityIndex.hh"

namespace gz::sim::ecs
{
  // Default-construct with as many worker threads as the hardware
  // reports. Falls back to 1 worker on systems where
  // `hardware_concurrency()` returns 0.
  World::World()
    : World(std::thread::hardware_concurrency())
  {
  }

  // Explicit thread-count constructor. Reserves command-buffer slot 0
  // for the main thread up front so the immediate path never has to
  // grow the buffer vector for the most common caller. Worker
  // threads register their own slots lazily on first
  // `LocalBuffer()` call.
  World::World(unsigned int _num_threads)
    : graph_(std::make_unique<ArchetypeGraph>()),
      entity_index_(std::make_unique<EntityIndex>()),
      worker_pool_(std::make_unique<gz::common::WorkerPool>(
          _num_threads == 0 ? 1u : _num_threads)),
      num_threads_(_num_threads == 0 ? 1 : _num_threads)
  {
    this->command_buffers_.push_back(std::make_unique<CommandBuffer>());
    this->thread_slots_.push_back({std::this_thread::get_id(), 0});
  }

  // Default destructor — `unique_ptr` members handle the cleanup.
  // Defined in the .cc so the header doesn't need full definitions
  // of `ArchetypeGraph`, `EntityIndex`, or `WorkerPool`.
  World::~World() = default;

  size_t World::NumEntities() const
  {
    return this->entity_index_->AliveCount();
  }

  size_t World::NumArchetypes() const
  {
    return this->graph_->Size();
  }

  bool World::IsAlive(Entity _e) const
  {
    return this->entity_index_->IsAlive(_e);
  }

  // True if the entity is alive AND its current archetype carries
  // the requested component type. Returns false for dead handles
  // and for live entities that don't have the component.
  bool World::Has(Entity _e, ComponentTypeId _id) const
  {
    if (!this->IsAlive(_e)) return false;
    auto rec = this->entity_index_->Get(_e);
    if (rec.archetype == kInvalidArchetypeId) return false;
    return this->graph_->Get(rec.archetype).Contains(_id);
  }

  // Per-thread override slot for parallel-mutation determinism.
  // EachParallel sets this to the worker's pre-assigned slot index
  // before invoking the user body, then clears it. With the override
  // active, LocalBuffer() bypasses the thread-id lookup and returns
  // the pre-assigned slot directly. Without this, slot assignment
  // would depend on which worker thread first called LocalBuffer,
  // which is scheduler-dependent — and that would make Commit's
  // replay order non-deterministic across runs.
  namespace
  {
    constexpr size_t kNoSlotOverride = static_cast<size_t>(-1);
    thread_local size_t tl_slot_override = kNoSlotOverride;
  }

  // RAII helper that EachParallel uses to scope a slot override.
  // The struct is declared in World.hh; bodies live here to keep
  // the thread_local symbol private to this TU.
  World::SlotOverrideGuard::SlotOverrideGuard(size_t _slot)
    : prev(tl_slot_override)
  {
    tl_slot_override = _slot;
  }
  World::SlotOverrideGuard::~SlotOverrideGuard()
  {
    tl_slot_override = prev;
  }

  // Vend the calling thread's command buffer. Two routes:
  //   1. If a parallel-walk slot override is active (set by
  //      EachParallel), return that slot directly. This is the
  //      fast deterministic path under EachParallel.
  //   2. Otherwise, look up by std::this_thread::get_id(),
  //      registering a new slot on first visit. Used by main-thread
  //      and ad-hoc threads outside EachParallel.
  CommandBuffer &World::LocalBuffer()
  {
    if (tl_slot_override != kNoSlotOverride)
    {
      std::lock_guard<std::mutex> lock(this->command_buffers_mutex_);
      return *this->command_buffers_[tl_slot_override];
    }
    auto tid = std::this_thread::get_id();
    {
      std::lock_guard<std::mutex> lock(this->command_buffers_mutex_);
      for (auto &slot : this->thread_slots_)
      {
        if (slot.id == tid)
          return *this->command_buffers_[slot.index];
      }
      // First visit from this thread — register a new slot.
      size_t idx = this->command_buffers_.size();
      this->command_buffers_.push_back(std::make_unique<CommandBuffer>());
      this->thread_slots_.push_back({tid, idx});
      return *this->command_buffers_[idx];
    }
  }

  // Reserve `_n` consecutive command-buffer slots starting at the
  // current end, and return the first slot's index. Called once
  // per EachParallel before the worker dispatch loop, so the slot
  // indices match the worker indices `0..n-1` in lockstep — that
  // pins Commit's merge order regardless of which worker actually
  // starts first on the OS scheduler.
  size_t World::ReserveBufferSlots(size_t _n)
  {
    std::lock_guard<std::mutex> lock(this->command_buffers_mutex_);
    size_t base = this->command_buffers_.size();
    for (size_t i = 0; i < _n; ++i)
      this->command_buffers_.push_back(std::make_unique<CommandBuffer>());
    return base;
  }

  // Direct buffer access by slot index. Internal; used by
  // EachParallel workers via the SlotOverrideGuard set up around
  // the user body. Locking is necessary because Commit may be
  // running concurrently on the main thread (in fact it isn't —
  // EachParallel is synchronous — but the mutex is cheap and
  // documents the invariant).
  CommandBuffer &World::BufferAtSlot(size_t _slot)
  {
    std::lock_guard<std::mutex> lock(this->command_buffers_mutex_);
    return *this->command_buffers_[_slot];
  }

  // Switch the world into deferred-mutation mode. Subsequent
  // `CreateEmpty` / `Destroy` / `AddRaw` / `RemoveRaw` calls queue
  // into the calling thread's command buffer instead of mutating
  // immediately. The queue drains at the next `Commit()`.
  //
  // SimulationRunner brackets each system phase (PreUpdate / Update
  // / PostUpdate) with `BeginPhase()` / `Commit()`, so plugin code
  // sees deferred semantics for the duration of a callback.
  void World::BeginPhase()
  {
    this->in_phase_ = true;
  }

  // Create a new entity with no components yet. Lands in the empty
  // archetype (ArchetypeId 0). Subsequent `AddRaw` calls move it
  // into other archetypes.
  //
  // In deferred mode, queues a CreateEntity command and returns
  // `kNullEntity` — the real handle isn't known until `Commit()`
  // replays the command. Callers that need the handle inside a
  // phase should arrange to receive it via the command's
  // `out_entity` field (used by templated `Create<T...>`).
  Entity World::CreateEmpty()
  {
    if (this->in_phase_)
    {
      auto &buf = this->LocalBuffer();
      Command cmd{};
      cmd.kind = CommandKind::CreateEntity;
      cmd.num_create_comps = 0;
      buf.Ops().push_back(cmd);
      return kNullEntity;
    }

    Entity e = this->entity_index_->Allocate();
    ArchetypeId aid = 0;  // empty archetype, allocated by ArchetypeGraph ctor
    Archetype &arch = this->graph_->Get(aid);
    auto [ci, row] = arch.AcquireRow();
    arch.ChunkAt(ci).Entities()[row] = e;
    this->entity_index_->Set(e, {aid, ci, row});
    arch.RecordNew(ci, row);
    return e;
  }

  // Single-step create-with-components. Looks up (or creates) the
  // archetype matching the pack's type set, reserves a row, copies
  // each component into its column, and registers the entity. This
  // is the common path used by `Commit()` when replaying a deferred
  // CreateEntity command, and also the one templated `Create<T...>`
  // calls inline outside of a phase.
  //
  // Idempotent on duplicate types in `_pack` — they're sorted and
  // deduplicated before lookup. Last write wins on duplicates,
  // since the dedup step keeps the first occurrence and the column
  // is copy-constructed once.
  Entity World::ImmediateCreateFromBlob(
      const std::vector<std::pair<const ComponentTypeInfo *, const void *>>
          &_pack)
  {
    std::vector<std::pair<const ComponentTypeInfo *, const void *>> pack =
        _pack;
    std::sort(pack.begin(), pack.end(),
        [](auto &a, auto &b) { return a.first->id < b.first->id; });
    pack.erase(std::unique(pack.begin(), pack.end(),
        [](auto &a, auto &b) { return a.first->id == b.first->id; }),
        pack.end());

    std::vector<ComponentTypeId> types;
    types.reserve(pack.size());
    for (auto &p : pack) types.push_back(p.first->id);

    ArchetypeId aid = this->graph_->GetOrCreate(types);
    Archetype &arch = this->graph_->Get(aid);

    auto [ci, row] = arch.AcquireRow();
    Chunk &chunk = arch.ChunkAt(ci);

    // Copy-construct each component value into its column slot.
    // The Archetype's column layout was computed at construction;
    // we just resolve the (column, row) byte offset and dispatch to
    // the type's `copy` hook.
    for (size_t i = 0; i < pack.size(); ++i)
    {
      auto *info = pack[i].first;
      const void *src = pack[i].second;
      size_t col_idx = arch.ColumnIndexOf(info->id);
      std::byte *col = chunk.ColumnBytes(arch.ColumnOffsets()[col_idx]);
      void *dst = col + row * info->size;
      info->copy(dst, src);
    }

    Entity e = this->entity_index_->Allocate();
    chunk.Entities()[row] = e;
    this->entity_index_->Set(e, {aid, ci, row});
    arch.RecordNew(ci, row);
    return e;
  }

  // Destroy an entity. In deferred mode, queues a DestroyEntity
  // command. In immediate mode, swaps the entity's row out of its
  // archetype (patching the displaced entity's index entry if any),
  // records a RemovedRecord for `EachRemoved` consumers, and frees
  // the entity slot (which bumps its generation, invalidating any
  // outstanding handles).
  void World::Destroy(Entity _e)
  {
    if (!this->IsAlive(_e))
      return;
    if (this->in_phase_)
    {
      auto &buf = this->LocalBuffer();
      Command cmd{};
      cmd.kind = CommandKind::DestroyEntity;
      cmd.entity = _e;
      buf.Ops().push_back(cmd);
      return;
    }

    auto rec = this->entity_index_->Get(_e);
    Archetype &arch = this->graph_->Get(rec.archetype);

    // RemovedRecord captures the type set as of just-before-removal
    // so EachRemoved consumers see what types this entity carried.
    RemovedRecord rr{_e, arch.Types()};
    this->removed_records_.push_back(std::move(rr));

    Entity moved = arch.SwapRemove(rec.chunk_idx, rec.row);
    if (moved != kNullEntity)
    {
      // Another row was relocated into the victim slot — patch its
      // index entry to point at the new (chunk, row).
      this->entity_index_->PatchByIndex(moved.Index(),
          {rec.archetype, rec.chunk_idx, rec.row});
    }
    this->entity_index_->Free(_e);
  }

  namespace
  {
    // Move every column shared between source and destination
    // archetypes from `(src, src_row)` into `(dst, dst_row)`. Used
    // by both Add (destination is a superset of source) and Remove
    // (destination is a subset).
    //
    // For trivially-relocatable types we use raw `memcpy` — bitwise
    // move is sound and skips destructor/copy-construct overhead. For
    // others, the type's `move` hook handles construction at the
    // destination; the source is destructed by the caller's later
    // `SwapRemove` of the source row.
    void MoveSharedColumns(Archetype &src_arch, Chunk &src_chunk,
                           uint32_t src_row,
                           Archetype &dst_arch, Chunk &dst_chunk,
                           uint32_t dst_row)
    {
      for (size_t si = 0; si < src_arch.Types().size(); ++si)
      {
        ComponentTypeId tid = src_arch.Types()[si];
        size_t di = dst_arch.ColumnIndexOf(tid);
        if (di == static_cast<size_t>(-1))
          continue;  // type isn't in destination — Remove case
        auto *info = src_arch.Infos()[si];
        std::byte *scol = src_chunk.ColumnBytes(
            src_arch.ColumnOffsets()[si]);
        std::byte *dcol = dst_chunk.ColumnBytes(
            dst_arch.ColumnOffsets()[di]);
        void *spos = scol + src_row * info->size;
        void *dpos = dcol + dst_row * info->size;
        if (info->trivially_relocatable)
          std::memcpy(dpos, spos, info->size);
        else
          info->move(dpos, spos);
      }
    }
  }

  // Add (or overwrite) a component on an entity. Two paths:
  //
  //   1. The entity's current archetype already carries `_info->id`
  //      → overwrite in place: destruct the existing value, copy-
  //      construct the new one, mark dirty. No archetype transition.
  //
  //   2. The entity needs a new column → look up the destination
  //      archetype (cached graph edge), reserve a row there, move
  //      every shared column across via `MoveSharedColumns`,
  //      copy-construct the new component into its destination
  //      column, swap-remove the source row, and patch the entity
  //      index.
  //
  // The `src_arch` reference is taken twice in the transition case
  // because `AddEdge` may allocate a new archetype and grow the
  // graph's internal vectors, invalidating any `Archetype&` taken
  // before the call.
  void World::ImmediateAdd(Entity _e, const ComponentTypeInfo *_info,
                           void *_src, bool /*_src_is_rvalue*/)
  {
    if (!this->IsAlive(_e)) return;

    auto rec = this->entity_index_->Get(_e);
    // Defensive: every alive entity owns a slot in the empty archetype
    // at minimum (`CreateEmpty` puts them there). If we ever observe
    // `kInvalidArchetypeId` here it's a bookkeeping bug; treat it as
    // "in archetype 0" rather than crash.
    if (rec.archetype == kInvalidArchetypeId)
      rec.archetype = 0;

    // Path 1: in-place overwrite.
    Archetype &src_arch = this->graph_->Get(rec.archetype);
    if (src_arch.Contains(_info->id))
    {
      size_t col_idx = src_arch.ColumnIndexOf(_info->id);
      Chunk &chunk = src_arch.ChunkAt(rec.chunk_idx);
      std::byte *col = chunk.ColumnBytes(src_arch.ColumnOffsets()[col_idx]);
      void *pos = col + rec.row * _info->size;
      _info->destruct(pos);
      _info->copy(pos, _src);
      src_arch.MarkDirty(col_idx, rec.chunk_idx, rec.row);
      return;
    }

    // Path 2: archetype transition. Capture the IDs first; both
    // src_arch and dst_arch are re-fetched after `AddEdge` because
    // that call may grow the graph's internal vectors.
    ArchetypeId src_aid = rec.archetype;
    ArchetypeId dst_aid = this->graph_->AddEdge(src_aid, _info->id);
    Archetype &src_arch_r = this->graph_->Get(src_aid);
    Archetype &dst_arch = this->graph_->Get(dst_aid);

    auto [dci, drow] = dst_arch.AcquireRow();
    Chunk &src_chunk = src_arch_r.ChunkAt(rec.chunk_idx);
    Chunk &dst_chunk = dst_arch.ChunkAt(dci);

    MoveSharedColumns(src_arch_r, src_chunk, rec.row,
                      dst_arch, dst_chunk, drow);

    // Place the freshly added component into its destination column.
    {
      size_t dcol_idx = dst_arch.ColumnIndexOf(_info->id);
      std::byte *col = dst_chunk.ColumnBytes(
          dst_arch.ColumnOffsets()[dcol_idx]);
      void *dpos = col + drow * _info->size;
      _info->copy(dpos, _src);
      dst_arch.MarkDirty(dcol_idx, dci, drow);
    }

    dst_chunk.Entities()[drow] = _e;
    dst_arch.RecordNew(dci, drow);

    // Vacate the source row. Any displaced row (the chunk's old
    // last) gets its index entry patched to point at the now-empty
    // slot it was moved into.
    Entity moved = src_arch_r.SwapRemove(rec.chunk_idx, rec.row);
    if (moved != kNullEntity)
    {
      this->entity_index_->PatchByIndex(moved.Index(),
          {src_aid, rec.chunk_idx, rec.row});
    }

    this->entity_index_->Set(_e, {dst_aid, dci, drow});
  }

  // Public typeId-keyed Remove. Routes through deferred or immediate
  // depending on whether a phase is open.
  void World::RemoveRaw(Entity _e, ComponentTypeId _id)
  {
    if (this->in_phase_) this->DeferredRemove(_e, _id);
    else                 this->ImmediateRemove(_e, _id);
  }

  // Public typeId-keyed Add. Looks up the type info from the
  // registry and forwards to `ImmediateAdd`. Silently no-ops if the
  // type isn't registered — the caller is responsible for ensuring
  // the type is known to the registry before this point.
  //
  // The const_cast is safe: the call chain (`ImmediateAdd` →
  // `info->copy(dst, src)`) treats `_src` as read-only. The non-const
  // `void*` in the signature is leftover from a previous design
  // iteration and should be tightened in a future cleanup.
  void World::AddRaw(Entity _e, ComponentTypeId _id, const void *_src)
  {
    auto *info = ComponentTypeRegistry::Instance().Get(_id);
    if (!info) return;
    this->ImmediateAdd(_e, info, const_cast<void *>(_src), false);
  }

  // Mirror of `ImmediateAdd` for the remove case. Looks up the
  // destination archetype (one that lacks `_id`), reserves a row
  // there, moves the kept columns across via `MoveSharedColumns`,
  // and swap-removes the source. Same re-fetch discipline applies.
  void World::ImmediateRemove(Entity _e, ComponentTypeId _id)
  {
    if (!this->IsAlive(_e)) return;
    auto rec = this->entity_index_->Get(_e);
    // Quick exit if the type isn't actually present. Read this
    // through a scoped reference so it doesn't outlive the
    // subsequent graph mutation.
    {
      Archetype &src_arch_check = this->graph_->Get(rec.archetype);
      if (!src_arch_check.Contains(_id)) return;
    }

    ArchetypeId src_aid = rec.archetype;
    ArchetypeId dst_aid = this->graph_->RemoveEdge(src_aid, _id);
    Archetype &src_arch_r = this->graph_->Get(src_aid);
    Archetype &dst_arch = this->graph_->Get(dst_aid);

    auto [dci, drow] = dst_arch.AcquireRow();
    Chunk &src_chunk = src_arch_r.ChunkAt(rec.chunk_idx);
    Chunk &dst_chunk = dst_arch.ChunkAt(dci);

    MoveSharedColumns(src_arch_r, src_chunk, rec.row,
                      dst_arch, dst_chunk, drow);

    dst_chunk.Entities()[drow] = _e;
    dst_arch.RecordNew(dci, drow);

    Entity moved = src_arch_r.SwapRemove(rec.chunk_idx, rec.row);
    if (moved != kNullEntity)
    {
      this->entity_index_->PatchByIndex(moved.Index(),
          {src_aid, rec.chunk_idx, rec.row});
    }

    this->entity_index_->Set(_e, {dst_aid, dci, drow});
  }

  // Encode an Add into the calling thread's command buffer. The
  // payload bytes are copied into the buffer's blob arena now (so
  // the caller can free `_src` immediately); the actual archetype
  // mutation runs at `Commit()` time via `ImmediateAdd`.
  void World::DeferredAdd(Entity _e, const ComponentTypeInfo *_info,
                          void *_src, bool /*_src_is_rvalue*/)
  {
    auto &buf = this->LocalBuffer();
    Command cmd{};
    cmd.kind = CommandKind::AddComponent;
    cmd.entity = _e;
    cmd.type = _info->id;
    cmd.payload_offset = buf.AppendPayload(_src, _info->size);
    cmd.payload_size = static_cast<uint32_t>(_info->size);
    buf.Ops().push_back(cmd);
  }

  // Encode a Remove. No payload — the type id alone is enough to
  // replay the operation in `Commit()`.
  void World::DeferredRemove(Entity _e, ComponentTypeId _id)
  {
    auto &buf = this->LocalBuffer();
    Command cmd{};
    cmd.kind = CommandKind::RemoveComponent;
    cmd.entity = _e;
    cmd.type = _id;
    buf.Ops().push_back(cmd);
  }

  // Read-only component access. Returns a const void pointing into
  // chunk storage. The pointer is valid until the next mutation
  // that crosses an archetype boundary on this entity (which, in
  // deferred mode, only happens at `Commit()`).
  //
  // Returns nullptr for dead entities or entities that don't carry
  // the requested type.
  const void *World::ComponentRaw(Entity _e, ComponentTypeId _id) const
  {
    if (!this->IsAlive(_e)) return nullptr;
    auto rec = this->entity_index_->Get(_e);
    Archetype &arch = this->graph_->Get(rec.archetype);
    size_t col_idx = arch.ColumnIndexOf(_id);
    if (col_idx == static_cast<size_t>(-1)) return nullptr;
    auto *info = arch.Infos()[col_idx];
    const std::byte *col = arch.ChunkAt(rec.chunk_idx)
        .ColumnBytes(arch.ColumnOffsets()[col_idx]);
    return col + rec.row * info->size;
  }

  // Mutable component access without dirty-bit marking. Used by the
  // facade's `ComponentImplementation(non-const)` path where the
  // caller is expected to be reading; if they actually mutate, they
  // should be going through `ComponentRawMutAndDirty` instead.
  void *World::ComponentRawMut(Entity _e, ComponentTypeId _id)
  {
    if (!this->IsAlive(_e)) return nullptr;
    auto rec = this->entity_index_->Get(_e);
    Archetype &arch = this->graph_->Get(rec.archetype);
    size_t col_idx = arch.ColumnIndexOf(_id);
    if (col_idx == static_cast<size_t>(-1)) return nullptr;
    auto *info = arch.Infos()[col_idx];
    std::byte *col = arch.ChunkAt(rec.chunk_idx)
        .ColumnBytes(arch.ColumnOffsets()[col_idx]);
    return col + rec.row * info->size;
  }

  // Mutable component access that also marks the (column, row) as
  // dirty. This is the "I am about to write" path — used by the
  // explicit `Modify<T>(e)` API and by any future ModifyGuard
  // proxy. EachChanged consumers see the row on next iteration.
  void *World::ComponentRawMutAndDirty(Entity _e, ComponentTypeId _id)
  {
    if (!this->IsAlive(_e)) return nullptr;
    auto rec = this->entity_index_->Get(_e);
    Archetype &arch = this->graph_->Get(rec.archetype);
    size_t col_idx = arch.ColumnIndexOf(_id);
    if (col_idx == static_cast<size_t>(-1)) return nullptr;
    auto *info = arch.Infos()[col_idx];
    std::byte *col = arch.ChunkAt(rec.chunk_idx)
        .ColumnBytes(arch.ColumnOffsets()[col_idx]);
    arch.MarkDirty(col_idx, rec.chunk_idx, rec.row);
    return col + rec.row * info->size;
  }

  // End of phase. Drains every per-thread command buffer in
  // deterministic order, applies the queued operations against the
  // live world state, and bumps the chunk-generation counter so
  // any cached `Component<T>(e)` pointer is observably invalid.
  //
  // Determinism guarantee: the command buffers are walked in slot
  // order (main-thread slot 0 first, then workers in the order they
  // first registered). Within each buffer, ops run in insertion
  // order. Two runs with the same step inputs and the same worker
  // count produce byte-identical state.
  //
  // Buffer-vector lifetime: we move `command_buffers_` out under
  // the mutex into a local, then re-seed slot 0 for the next
  // phase. This decouples the drain from any new buffers the
  // post-Commit code might allocate.
  void World::Commit()
  {
    std::vector<std::unique_ptr<CommandBuffer>> locals;
    {
      std::lock_guard<std::mutex> lock(this->command_buffers_mutex_);
      locals = std::move(this->command_buffers_);
      this->command_buffers_.clear();
      this->command_buffers_.push_back(std::make_unique<CommandBuffer>());
      // Worker threads will re-register their slots lazily on
      // first LocalBuffer() call in the next phase.
      this->thread_slots_.clear();
      this->thread_slots_.push_back({std::this_thread::get_id(), 0});
    }

    for (auto &buf_ptr : locals)
    {
      if (!buf_ptr) continue;
      auto &buf = *buf_ptr;
      // CreateEntity records carry per-component metadata in a
      // separate vector on the buffer. `create_cursor` walks that
      // vector in lockstep with the CreateEntity ops we encounter.
      size_t create_cursor = 0;
      for (const auto &cmd : buf.Ops())
      {
        switch (cmd.kind)
        {
          case CommandKind::CreateEntity:
          {
            // Reconstruct the (info, payload) pack and forward to
            // the immediate path, then fan the resulting handle
            // back through `out_entity` if the caller wired one up.
            auto &reg = ComponentTypeRegistry::Instance();
            std::vector<std::pair<const ComponentTypeInfo *, const void *>>
                pack;
            pack.reserve(cmd.num_create_comps);
            const auto &comps = buf.CreateComps();
            const std::byte *blob = buf.Blob().data();
            for (uint32_t i = 0; i < cmd.num_create_comps; ++i)
            {
              auto &cc = comps[create_cursor + i];
              const ComponentTypeInfo *info = reg.Get(cc.type);
              assert(info != nullptr);
              pack.emplace_back(info, blob + cc.payload_offset);
            }
            create_cursor += cmd.num_create_comps;
            Entity created = this->ImmediateCreateFromBlob(pack);
            if (cmd.out_entity) *cmd.out_entity = created;
            break;
          }
          case CommandKind::DestroyEntity:
          {
            // Re-check IsAlive — a prior op in this commit batch
            // may have already destroyed the entity (idempotent
            // multi-destroy is supported).
            if (this->IsAlive(cmd.entity))
            {
              auto rec = this->entity_index_->Get(cmd.entity);
              this->removed_records_.push_back({cmd.entity,
                  this->graph_->Get(rec.archetype).Types()});
              Archetype &arch = this->graph_->Get(rec.archetype);
              Entity moved = arch.SwapRemove(rec.chunk_idx, rec.row);
              if (moved != kNullEntity)
              {
                this->entity_index_->PatchByIndex(moved.Index(),
                    {rec.archetype, rec.chunk_idx, rec.row});
              }
              this->entity_index_->Free(cmd.entity);
            }
            break;
          }
          case CommandKind::AddComponent:
          {
            const ComponentTypeInfo *info =
                ComponentTypeRegistry::Instance().Get(cmd.type);
            assert(info != nullptr);
            void *src = const_cast<std::byte *>(
                buf.Blob().data() + cmd.payload_offset);
            this->ImmediateAdd(cmd.entity, info, src, true);
            // ImmediateAdd copy-constructed from `src`; destruct
            // the staged value to balance the construction in
            // `DeferredAdd`/`AppendPayload`.
            info->destruct(src);
            break;
          }
          case CommandKind::RemoveComponent:
          {
            this->ImmediateRemove(cmd.entity, cmd.type);
            break;
          }
          case CommandKind::SetComponent:
          {
            // SetComponent is currently identical to AddComponent
            // (both overwrite). The two kinds exist so a future
            // refinement can distinguish "must already exist" from
            // "create-or-overwrite" semantics without an API break.
            const ComponentTypeInfo *info =
                ComponentTypeRegistry::Instance().Get(cmd.type);
            assert(info != nullptr);
            void *src = const_cast<std::byte *>(
                buf.Blob().data() + cmd.payload_offset);
            this->ImmediateAdd(cmd.entity, info, src, true);
            info->destruct(src);
            break;
          }
        }
      }
    }

    this->in_phase_ = false;
    // Bump the generation counter — any pointer obtained from
    // `Component<T>` before this commit is now considered invalid.
    ++this->chunk_generation_;
  }

  // End-of-step sweep: zero every dirty bit, drop the
  // newly-added-rows record, and forget every removed entity. Run
  // by SimulationRunner once per step, after the final Commit().
  void World::ClearChangeBits()
  {
    this->graph_->ForEach([](Archetype &_a)
    {
      _a.ClearDirtyBits();
      _a.ClearNewlyAdded();
    });
    this->removed_records_.clear();
  }
}
