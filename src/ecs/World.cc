/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
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
  World::World()
    : World(std::thread::hardware_concurrency())
  {
  }

  World::World(unsigned int _num_threads)
    : graph_(std::make_unique<ArchetypeGraph>()),
      entity_index_(std::make_unique<EntityIndex>()),
      worker_pool_(std::make_unique<gz::common::WorkerPool>(
          _num_threads == 0 ? 1u : _num_threads)),
      num_threads_(_num_threads == 0 ? 1 : _num_threads)
  {
    // Main thread gets slot 0.
    this->command_buffers_.push_back(std::make_unique<CommandBuffer>());
    this->thread_slots_.push_back({std::this_thread::get_id(), 0});
  }

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

  bool World::Has(Entity _e, ComponentTypeId _id) const
  {
    if (!this->IsAlive(_e)) return false;
    auto rec = this->entity_index_->Get(_e);
    if (rec.archetype == kInvalidArchetypeId) return false;
    return this->graph_->Get(rec.archetype).Contains(_id);
  }

  CommandBuffer &World::LocalBuffer()
  {
    auto tid = std::this_thread::get_id();
    // Fast path: linear scan, usually 1–N entries.
    {
      std::lock_guard<std::mutex> lock(this->command_buffers_mutex_);
      for (auto &slot : this->thread_slots_)
      {
        if (slot.id == tid)
          return *this->command_buffers_[slot.index];
      }
      size_t idx = this->command_buffers_.size();
      this->command_buffers_.push_back(std::make_unique<CommandBuffer>());
      this->thread_slots_.push_back({tid, idx});
      return *this->command_buffers_[idx];
    }
  }

  void World::BeginPhase()
  {
    this->in_phase_ = true;
  }

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
    ArchetypeId aid = 0;  // empty archetype
    Archetype &arch = this->graph_->Get(aid);
    auto [ci, row] = arch.AcquireRow();
    arch.ChunkAt(ci).Entities()[row] = e;
    this->entity_index_->Set(e, {aid, ci, row});
    arch.RecordNew(ci, row);
    return e;
  }

  Entity World::ImmediateCreateFromBlob(
      const std::vector<std::pair<const ComponentTypeInfo *, const void *>>
          &_pack)
  {
    // Sort types and deduplicate (caller may pass duplicates).
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

    // Copy-construct each component into its column.
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

    // Capture a removed record for EachRemoved.
    RemovedRecord rr{_e, arch.Types()};
    this->removed_records_.push_back(std::move(rr));

    Entity moved = arch.SwapRemove(rec.chunk_idx, rec.row);
    if (moved != kNullEntity)
    {
      this->entity_index_->PatchByIndex(moved.Index(),
          {rec.archetype, rec.chunk_idx, rec.row});
    }
    this->entity_index_->Free(_e);
  }

  namespace
  {
    // Move all shared columns from (src_arch, src_chunk, src_row) into
    // (dst_arch, dst_chunk, dst_row).
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
          continue;  // not in destination (Remove case)
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

  void World::ImmediateAdd(Entity _e, const ComponentTypeInfo *_info,
                           void *_src, bool /*_src_is_rvalue*/)
  {
    if (!this->IsAlive(_e)) return;

    auto rec = this->entity_index_->Get(_e);
    // Fresh entity that never occupied the empty archetype — should not
    // happen in 0a; every entity starts in archetype 0.
    if (rec.archetype == kInvalidArchetypeId)
      rec.archetype = 0;

    // If already present, just overwrite in place.
    Archetype &src_arch = this->graph_->Get(rec.archetype);
    if (src_arch.Contains(_info->id))
    {
      size_t col_idx = src_arch.ColumnIndexOf(_info->id);
      Chunk &chunk = src_arch.ChunkAt(rec.chunk_idx);
      std::byte *col = chunk.ColumnBytes(src_arch.ColumnOffsets()[col_idx]);
      void *pos = col + rec.row * _info->size;
      // destruct + copy-construct.
      _info->destruct(pos);
      _info->copy(pos, _src);
      src_arch.MarkDirty(col_idx, rec.chunk_idx, rec.row);
      return;
    }

    // AddEdge may allocate a new archetype and grow the graph's internal
    // vectors — invalidating src_arch. Take the id, then re-fetch.
    ArchetypeId src_aid = rec.archetype;
    ArchetypeId dst_aid = this->graph_->AddEdge(src_aid, _info->id);
    Archetype &src_arch_r = this->graph_->Get(src_aid);
    Archetype &dst_arch = this->graph_->Get(dst_aid);

    auto [dci, drow] = dst_arch.AcquireRow();
    Chunk &src_chunk = src_arch_r.ChunkAt(rec.chunk_idx);
    Chunk &dst_chunk = dst_arch.ChunkAt(dci);

    MoveSharedColumns(src_arch_r, src_chunk, rec.row,
                      dst_arch, dst_chunk, drow);

    // Place the new component in its destination column.
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

    Entity moved = src_arch_r.SwapRemove(rec.chunk_idx, rec.row);
    if (moved != kNullEntity)
    {
      this->entity_index_->PatchByIndex(moved.Index(),
          {src_aid, rec.chunk_idx, rec.row});
    }

    this->entity_index_->Set(_e, {dst_aid, dci, drow});
  }

  void World::RemoveRaw(Entity _e, ComponentTypeId _id)
  {
    if (this->in_phase_) this->DeferredRemove(_e, _id);
    else                 this->ImmediateRemove(_e, _id);
  }

  void World::ImmediateRemove(Entity _e, ComponentTypeId _id)
  {
    if (!this->IsAlive(_e)) return;
    auto rec = this->entity_index_->Get(_e);
    // Read needed data from src_arch before any graph mutation.
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

  void World::DeferredRemove(Entity _e, ComponentTypeId _id)
  {
    auto &buf = this->LocalBuffer();
    Command cmd{};
    cmd.kind = CommandKind::RemoveComponent;
    cmd.entity = _e;
    cmd.type = _id;
    buf.Ops().push_back(cmd);
  }

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

  void World::Commit()
  {
    std::vector<std::unique_ptr<CommandBuffer>> locals;
    {
      std::lock_guard<std::mutex> lock(this->command_buffers_mutex_);
      locals = std::move(this->command_buffers_);
      // Refresh main buffer for the next phase. Worker slots will be
      // re-populated lazily on first use.
      this->command_buffers_.clear();
      this->command_buffers_.push_back(std::make_unique<CommandBuffer>());
      // Slot 0 stays reserved for the main thread.
      this->thread_slots_.clear();
      this->thread_slots_.push_back({std::this_thread::get_id(), 0});
    }

    // Deterministic merge order: slot 0 first, then sorted by slot index.
    // Since we appended in monotonic order, the vector order is already the
    // merge order.
    for (auto &buf_ptr : locals)
    {
      if (!buf_ptr) continue;
      auto &buf = *buf_ptr;
      size_t create_cursor = 0;
      for (const auto &cmd : buf.Ops())
      {
        switch (cmd.kind)
        {
          case CommandKind::CreateEntity:
          {
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
            // Stash removal record.
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
            // destruct the staged value — we copy-constructed out of it.
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
            // 0a: not distinct from AddComponent in behaviour; overwrite.
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
    ++this->chunk_generation_;
  }

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
