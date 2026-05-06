/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_ARCHETYPE_HH_
#define GZ_SIM_ECS_ARCHETYPE_HH_

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "gz/sim/ecs/ComponentTypeRegistry.hh"
#include "gz/sim/ecs/Entity.hh"
#include "gz/sim/ecs/detail/Chunk.hh"
#include "gz/sim/ecs/detail/EntityIndex.hh"

namespace gz::sim::ecs
{
  /// \brief An Archetype identifies a group of entities that share exactly
  /// the same set of component types. It owns a list of Chunks of SoA
  /// column storage.
  class Archetype
  {
    public: Archetype(ArchetypeId _id,
                     std::vector<ComponentTypeId> _types,
                     std::vector<const ComponentTypeInfo *> _infos,
                     size_t _chunk_size = Chunk::kDefaultSize);
    public: ~Archetype();

    public: Archetype(const Archetype &) = delete;
    public: Archetype &operator=(const Archetype &) = delete;

    public: ArchetypeId Id() const { return this->id_; }

    public: const std::vector<ComponentTypeId> &Types() const
    { return this->types_; }

    public: const std::vector<const ComponentTypeInfo *> &Infos() const
    { return this->infos_; }

    public: const std::vector<size_t> &ColumnOffsets() const
    { return this->col_offsets_; }

    public: size_t ChunkCapacity() const { return this->chunk_capacity_; }
    public: size_t ChunkSize()     const { return this->chunk_size_;     }

    public: size_t ColumnIndexOf(ComponentTypeId _id) const;
    public: bool   Contains(ComponentTypeId _id) const;

    /// \brief Number of chunks currently allocated.
    public: size_t NumChunks() const { return this->chunks_.size(); }

    /// \brief Access a chunk by index.
    public: Chunk &ChunkAt(size_t _i) { return *this->chunks_[_i]; }
    public: const Chunk &ChunkAt(size_t _i) const { return *this->chunks_[_i]; }

    /// \brief Acquire an empty row somewhere in the archetype. Returns
    /// (chunk_idx, row). Allocates a new chunk if all existing chunks are
    /// full. Does NOT construct into the columns — caller must do that.
    public: std::pair<uint32_t, uint32_t> AcquireRow();

    /// \brief Run destructors on row (chunk_idx, row) for every column in
    /// the archetype. Does NOT adjust entity column / row counts.
    public: void DestructRow(uint32_t _chunk_idx, uint32_t _row);

    /// \brief Remove a row via swap-with-last. Returns the Entity whose row
    /// was moved into the vacated slot (or kNullEntity if the removed row
    /// was the last row in the chunk). Caller is responsible for patching
    /// that entity's EntityIndex slot to the new (chunk_idx, _row).
    public: Entity SwapRemove(uint32_t _chunk_idx, uint32_t _row);

    /// \brief Number of live rows across all chunks.
    public: size_t TotalRows() const;

    /// \brief Clear all dirty bits across all chunks.
    public: void ClearDirtyBits();

    /// \brief Mark a component's dirty bit for a row.
    public: void MarkDirty(size_t _col_idx, uint32_t _chunk_idx, uint32_t _row);

    /// \brief Read a dirty bit.
    public: bool IsDirty(size_t _col_idx,
                         uint32_t _chunk_idx,
                         uint32_t _row) const;

    /// \brief Append-only log of rows added this step (for EachNew).
    public: struct NewRecord
    {
      uint32_t chunk_idx;
      uint32_t row;
    };
    public: const std::vector<NewRecord> &NewlyAdded() const
    { return this->newly_added_; }
    public: void RecordNew(uint32_t _chunk_idx, uint32_t _row)
    { this->newly_added_.push_back({_chunk_idx, _row}); }
    public: void ClearNewlyAdded() { this->newly_added_.clear(); }

    private: size_t DirtyWordsPerColumn() const;

    private: ArchetypeId                            id_;
    private: std::vector<ComponentTypeId>           types_;
    private: std::vector<const ComponentTypeInfo *> infos_;
    private: std::vector<size_t>                    col_offsets_;
    private: size_t                                 row_size_{0};
    private: size_t                                 chunk_capacity_{0};
    private: size_t                                 chunk_size_{0};
    private: std::vector<std::unique_ptr<Chunk>>    chunks_;
    // Stack of chunk indices known to have at least one free row.
    // Pushed by SwapRemove (a row was freed); popped by AcquireRow.
    // The "tail" chunk (chunks_.back()) is handled separately in
    // AcquireRow without going through this stack — that's the
    // hot path for the build workload (no removals).
    private: std::vector<uint32_t>                  free_chunk_stack_;
    private: std::vector<NewRecord>                 newly_added_;
  };
}

#endif
