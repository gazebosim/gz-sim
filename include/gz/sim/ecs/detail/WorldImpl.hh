/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
// Template implementation for gz::sim::ecs::World. Included from World.hh.
#ifndef GZ_SIM_ECS_DETAIL_WORLDIMPL_HH_
#define GZ_SIM_ECS_DETAIL_WORLDIMPL_HH_

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <gz/common/WorkerPool.hh>

#include "gz/sim/ecs/detail/Archetype.hh"
#include "gz/sim/ecs/detail/ArchetypeGraph.hh"
#include "gz/sim/ecs/detail/Chunk.hh"
#include "gz/sim/ecs/detail/EntityIndex.hh"

namespace gz::sim::ecs
{
  namespace detail
  {
    template <class... Cs>
    inline std::array<ComponentTypeId, sizeof...(Cs)> RequiredIdsOf()
    {
      auto &reg = ComponentTypeRegistry::Instance();
      return {reg.template Register<std::remove_const_t<Cs>>()->id...};
    }

    /// \brief Resolve column indices for each Cs... within `_a`.
    template <class... Cs>
    inline std::array<size_t, sizeof...(Cs)> ColumnIndicesOf(
        const Archetype &_a)
    {
      auto &reg = ComponentTypeRegistry::Instance();
      return {_a.ColumnIndexOf(
          reg.template TypeIdOf<std::remove_const_t<Cs>>())...};
    }

    /// \brief Compute per-column base pointers in a chunk given precomputed
    /// column indices into the archetype.
    template <std::size_t N>
    inline std::array<void *, N> ColumnPointers(
        Archetype &_a, Chunk &_c,
        const std::array<size_t, N> &_col_idxs)
    {
      std::array<void *, N> out{};
      for (std::size_t i = 0; i < N; ++i)
        out[i] = _c.ColumnBytes(_a.ColumnOffsets()[_col_idxs[i]]);
      return out;
    }

    /// \brief fn(e, col0[row], col1[row], ...) for a single row.
    /// Used by per-row paths (EachNew, EachChanged) where we walk
    /// a sparse subset of rows.
    template <class Fn, class... Cs, std::size_t... Is>
    inline void InvokeRow(Fn &_fn, Entity _e,
                          const std::array<void *, sizeof...(Cs)> &_cols,
                          uint32_t _row, std::index_sequence<Is...>)
    {
      _fn(_e,
          static_cast<std::add_pointer_t<std::remove_reference_t<Cs>>>(
              _cols[Is])[_row]...);
    }

    /// \brief Per-chunk inner loop. Typed column pointers are passed
    /// as named function parameters (a parameter pack), not stuffed
    /// into a tuple — this keeps them in registers across the row
    /// loop and lets the compiler hoist any lambda-internal
    /// loop-invariants (e.g. literal constants) out of the loop body.
    template <class Fn, class... Cs>
    [[gnu::always_inline]] inline void InvokeChunkPacked(
        Fn &_fn, const Entity *_ents, uint32_t _count,
        std::add_pointer_t<std::remove_reference_t<Cs>>... _typed)
    {
      // ivdep tells GCC the row iterations are independent (columns
      // never alias by archetype construction). This unlocks SSE2
      // vectorization of the lambda body — measurable but small win
      // because the iteration is already memory-bandwidth bound.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC ivdep
#endif
      for (uint32_t r = 0; r < _count; ++r)
      {
        _fn(_ents[r], _typed[r]...);
      }
    }

    /// \brief Trampoline that expands the pack via index_sequence and
    /// hands typed pointers to InvokeChunkPacked as named parameters.
    template <class Fn, class... Cs, std::size_t... Is>
    [[gnu::always_inline]] inline void InvokeChunkImpl(
        Fn &_fn, const Entity *_ents,
        const std::array<void *, sizeof...(Cs)> &_cols, uint32_t _count,
        std::index_sequence<Is...>)
    {
      InvokeChunkPacked<Fn, Cs...>(_fn, _ents, _count,
          static_cast<std::add_pointer_t<std::remove_reference_t<Cs>>>(
              _cols[Is])...);
    }

    template <class Fn, class... Cs>
    [[gnu::always_inline]] inline void InvokeChunk(
        Fn &_fn, Entity *_ents,
        const std::array<void *, sizeof...(Cs)> &_cols,
        uint32_t _count)
    {
      InvokeChunkImpl<Fn, Cs...>(_fn, _ents, _cols, _count,
          std::index_sequence_for<Cs...>{});
    }
  }

  template <class... Cs, class Fn>
  void World::Each(Fn &&_fn)
  {
    thread_local Query tl_query(
        std::vector<ComponentTypeId>{
            ComponentTypeRegistry::Instance().TypeIdOf<
                std::remove_const_t<Cs>>()...});
    tl_query.RefreshIfStale(*this);

    for (ArchetypeId aid : tl_query.Matching())
    {
      Archetype &arch = this->Graph().Get(aid);
      auto col_idxs = detail::ColumnIndicesOf<Cs...>(arch);

      for (size_t ci = 0; ci < arch.NumChunks(); ++ci)
      {
        Chunk &chunk = arch.ChunkAt(ci);
        const uint32_t count = chunk.Count();
        if (count == 0) continue;
        auto cols = detail::ColumnPointers(arch, chunk, col_idxs);
        detail::InvokeChunk<Fn, Cs...>(_fn, chunk.Entities(), cols, count);
      }
    }
  }

  template <class... Cs, class Fn>
  void World::EachParallel(Fn &&_fn)
  {
    thread_local Query tl_query(
        std::vector<ComponentTypeId>{
            ComponentTypeRegistry::Instance().TypeIdOf<
                std::remove_const_t<Cs>>()...});
    tl_query.RefreshIfStale(*this);

    struct Unit { ArchetypeId arch; uint32_t chunk; };
    std::vector<Unit> units;
    for (ArchetypeId aid : tl_query.Matching())
    {
      Archetype &arch = this->Graph().Get(aid);
      for (size_t ci = 0; ci < arch.NumChunks(); ++ci)
        units.push_back({aid, static_cast<uint32_t>(ci)});
    }
    if (units.empty()) return;

    unsigned int threads = this->NumThreads();
    if (threads <= 1 || units.size() < threads)
    {
      for (auto u : units)
      {
        Archetype &arch = this->Graph().Get(u.arch);
        auto col_idxs = detail::ColumnIndicesOf<Cs...>(arch);
        Chunk &chunk = arch.ChunkAt(u.chunk);
        const uint32_t count = chunk.Count();
        if (count == 0) continue;
        auto cols = detail::ColumnPointers(arch, chunk, col_idxs);
        detail::InvokeChunk<Fn, Cs...>(_fn, chunk.Entities(), cols, count);
      }
      return;
    }

    const size_t per = (units.size() + threads - 1) / threads;
    auto *pool = this->WorkerPoolInternal();
    auto *world = this;

    // Determinism: pre-reserve `threads` command-buffer slots in
    // a deterministic loop, BEFORE dispatching work. Each worker
    // index t binds to slot (base_slot + t) via the
    // SlotOverrideGuard. Without this, slot order would depend on
    // which worker thread first calls LocalBuffer — scheduler-
    // dependent — and Commit's replay order would vary across
    // runs even with identical inputs and worker counts.
    const size_t base_slot = world->ReserveBufferSlots(threads);

    for (unsigned int t = 0; t < threads; ++t)
    {
      size_t lo = t * per;
      size_t hi = std::min(lo + per, units.size());
      if (lo >= hi) break;
      const size_t slot = base_slot + t;
      pool->AddWork([world, &units, &_fn, lo, hi, slot]()
      {
        World::SlotOverrideGuard guard(slot);
        for (size_t i = lo; i < hi; ++i)
        {
          auto u = units[i];
          Archetype &arch = world->Graph().Get(u.arch);
          auto col_idxs = detail::ColumnIndicesOf<Cs...>(arch);
          Chunk &chunk = arch.ChunkAt(u.chunk);
          const uint32_t count = chunk.Count();
          if (count == 0) continue;
          auto cols = detail::ColumnPointers(arch, chunk, col_idxs);
          detail::InvokeChunk<Fn, Cs...>(
              const_cast<Fn &>(_fn), chunk.Entities(), cols, count);
        }
      });
    }
    pool->WaitForResults();
  }

  template <class... Cs, class Fn>
  void World::EachNew(Fn &&_fn)
  {
    thread_local Query tl_query(
        std::vector<ComponentTypeId>{
            ComponentTypeRegistry::Instance().TypeIdOf<
                std::remove_const_t<Cs>>()...});
    tl_query.RefreshIfStale(*this);

    for (ArchetypeId aid : tl_query.Matching())
    {
      Archetype &arch = this->Graph().Get(aid);
      if (arch.NewlyAdded().empty()) continue;
      auto col_idxs = detail::ColumnIndicesOf<Cs...>(arch);

      for (auto rec : arch.NewlyAdded())
      {
        Chunk &chunk = arch.ChunkAt(rec.chunk_idx);
        if (rec.row >= chunk.Count()) continue;
        auto cols = detail::ColumnPointers(arch, chunk, col_idxs);
        detail::InvokeRow<Fn, Cs...>(_fn, chunk.Entities()[rec.row],
            cols, rec.row, std::index_sequence_for<Cs...>{});
      }
    }
  }

  template <class... Cs, class Fn>
  void World::EachChanged(Fn &&_fn)
  {
    thread_local Query tl_query(
        std::vector<ComponentTypeId>{
            ComponentTypeRegistry::Instance().TypeIdOf<
                std::remove_const_t<Cs>>()...});
    tl_query.RefreshIfStale(*this);

    for (ArchetypeId aid : tl_query.Matching())
    {
      Archetype &arch = this->Graph().Get(aid);
      auto col_idxs = detail::ColumnIndicesOf<Cs...>(arch);

      for (size_t ci = 0; ci < arch.NumChunks(); ++ci)
      {
        Chunk &chunk = arch.ChunkAt(ci);
        const uint32_t count = chunk.Count();
        if (count == 0) continue;
        auto cols = detail::ColumnPointers(arch, chunk, col_idxs);
        Entity *ents = chunk.Entities();
        for (uint32_t r = 0; r < count; ++r)
        {
          bool any = false;
          for (size_t k = 0; k < col_idxs.size(); ++k)
          {
            if (arch.IsDirty(col_idxs[k],
                             static_cast<uint32_t>(ci), r))
            {
              any = true; break;
            }
          }
          if (!any) continue;
          detail::InvokeRow<Fn, Cs...>(_fn, ents[r], cols, r,
              std::index_sequence_for<Cs...>{});
        }
      }
    }
  }

  template <class... Cs, class Fn>
  void World::EachRemoved(Fn &&_fn)
  {
    const auto required = detail::RequiredIdsOf<Cs...>();
    for (const auto &rec : this->RemovedRecords())
    {
      bool all = true;
      for (auto t : required)
      {
        if (std::find(rec.types.begin(), rec.types.end(), t) ==
            rec.types.end())
        {
          all = false; break;
        }
      }
      if (!all) continue;
      _fn(rec.entity);
    }
  }

  template <class... Cs>
  Entity World::DeferredCreate(Cs &&... _cs)
  {
    auto &buf = this->LocalBuffer();
    auto &reg = ComponentTypeRegistry::Instance();

    Command cmd{};
    cmd.kind = CommandKind::CreateEntity;
    cmd.num_create_comps = sizeof...(Cs);

    auto stage = [&](auto &&c)
    {
      using T = std::decay_t<decltype(c)>;
      auto *info = reg.template Register<T>();
      uint32_t off = buf.AppendPayload(std::addressof(c), sizeof(T));
      buf.CreateComps().push_back(
          {info->id, off, static_cast<uint32_t>(sizeof(T))});
    };
    (stage(std::forward<Cs>(_cs)), ...);

    buf.Ops().push_back(cmd);
    return kNullEntity;
  }
}

#endif
