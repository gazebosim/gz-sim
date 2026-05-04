/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// Archetype — owner of one component-type set's chunked SoA storage.
//
// An archetype groups every entity that has *exactly* the same set of
// component types. It owns:
//   * The sorted, deduplicated list of `ComponentTypeId`s that defines
//     the group.
//   * Per-column byte offsets within a chunk, computed once at
//     construction by `ComputeColumnLayout` (alignment-aware).
//   * A list of chunks (16 KiB each by default), filled in order;
//     allocation spills into a fresh chunk when the last one is full.
//   * Change-tracking dirty bits, packed one bit per (row, column).
//
// The hot operation is `SwapRemove`: when an entity is destroyed or
// moved to a different archetype, its row is overwritten by the
// archetype's last row and the row counter shrinks. This keeps the
// chunks dense (no holes) so iteration stays branch-free, at the cost
// of one entity-index patch on the entity that was relocated.
#include "gz/sim/ecs/detail/Archetype.hh"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <stdexcept>

#include "gz/sim/ecs/detail/ColumnLayout.hh"

namespace gz::sim::ecs
{
  // The constructor establishes the chunk layout exactly once. After
  // this runs, a row is just `chunk.bytes + col_offsets_[col] + row *
  // info->size` — no per-access computation.
  //
  // Type vector invariant: sorted ascending and deduplicated. The
  // ArchetypeGraph guarantees this, but we assert it here as well so
  // a buggy caller fails loudly in debug builds.
  Archetype::Archetype(ArchetypeId _id,
                       std::vector<ComponentTypeId> _types,
                       std::vector<const ComponentTypeInfo *> _infos,
                       size_t _chunk_size)
    : id_(_id),
      types_(std::move(_types)),
      infos_(std::move(_infos)),
      chunk_size_(_chunk_size)
  {
    assert(this->types_.size() == this->infos_.size());
    for (size_t i = 1; i < this->types_.size(); ++i)
      assert(this->types_[i - 1] < this->types_[i]);

    auto layout = detail::ComputeColumnLayout(this->infos_, _chunk_size);
    if (layout.capacity == 0)
    {
      // A single row didn't fit in `_chunk_size` bytes after alignment.
      // The default chunk size (16 KiB) handles every realistic
      // component set; if a user component is large enough to trip
      // this they need a bigger chunk size.
      throw std::runtime_error(
          "Archetype: row too large for configured chunk size");
    }
    this->col_offsets_ = std::move(layout.offsets);
    this->chunk_capacity_ = layout.capacity;

    size_t row_bytes = sizeof(Entity);
    for (auto *info : this->infos_)
      row_bytes += info->size;
    this->row_size_ = row_bytes;
  }

  // Chunks own raw bytes — they don't know which columns they hold or
  // how to destruct them. The archetype carries that knowledge, so it
  // is responsible for destructing every live row before the chunk's
  // storage is freed.
  Archetype::~Archetype()
  {
    for (auto &chunk_ptr : this->chunks_)
    {
      Chunk &chunk = *chunk_ptr;
      for (uint32_t row = 0; row < chunk.Count(); ++row)
      {
        for (size_t c = 0; c < this->infos_.size(); ++c)
        {
          auto *info = this->infos_[c];
          std::byte *col = chunk.ColumnBytes(this->col_offsets_[c]);
          info->destruct(col + row * info->size);
        }
      }
    }
  }

  // Find the column index for `_id`. Returns `(size_t)-1` if this
  // archetype doesn't carry that type. Binary search on the sorted
  // type vector — typically 4–10 types per archetype, so this is a
  // handful of comparisons.
  size_t Archetype::ColumnIndexOf(ComponentTypeId _id) const
  {
    auto it = std::lower_bound(
        this->types_.begin(), this->types_.end(), _id);
    if (it == this->types_.end() || *it != _id)
      return static_cast<size_t>(-1);
    return static_cast<size_t>(it - this->types_.begin());
  }

  // True when this archetype's type set includes `_id`.
  bool Archetype::Contains(ComponentTypeId _id) const
  {
    return this->ColumnIndexOf(_id) != static_cast<size_t>(-1);
  }

  // Reserve a row in some chunk and return `(chunk_idx, row)` for
  // the caller to populate. Walks the existing chunks in order
  // looking for a non-full one; allocates a new chunk if none has
  // space. The fresh chunk gets its dirty-bit bitset sized to
  // `(rows * columns + 63) / 64` 64-bit words.
  std::pair<uint32_t, uint32_t> Archetype::AcquireRow()
  {
    for (size_t i = 0; i < this->chunks_.size(); ++i)
    {
      if (!this->chunks_[i]->Full())
      {
        uint32_t row = this->chunks_[i]->AcquireRow();
        return {static_cast<uint32_t>(i), row};
      }
    }

    auto chunk = std::make_unique<Chunk>(
        this->chunk_size_, this->chunk_capacity_);

    size_t words_per_col = (this->chunk_capacity_ + 63) / 64;
    chunk->AllocDirtyBits(words_per_col * this->infos_.size());

    uint32_t row = chunk->AcquireRow();
    this->chunks_.push_back(std::move(chunk));
    return {static_cast<uint32_t>(this->chunks_.size() - 1), row};
  }

  // Run every column's destructor on the (chunk, row) tuple. Does
  // not adjust the chunk's row count — `SwapRemove` calls this and
  // then decides whether to pop the row or fill it with the last row.
  void Archetype::DestructRow(uint32_t _chunk_idx, uint32_t _row)
  {
    Chunk &chunk = *this->chunks_[_chunk_idx];
    for (size_t c = 0; c < this->infos_.size(); ++c)
    {
      auto *info = this->infos_[c];
      std::byte *col = chunk.ColumnBytes(this->col_offsets_[c]);
      info->destruct(col + _row * info->size);
    }
  }

  // Remove a row by overwriting it with the chunk's last row, then
  // shrinking the row count by one. Returns the entity that was
  // relocated (so the caller can patch the entity index), or
  // `kNullEntity` when the removed row was already the last.
  //
  // Why "swap" instead of "shift": shifting is O(n) per remove;
  // swap-with-last is O(1) and keeps the chunks dense for the linear
  // iteration that the `Each` hot loop depends on. The order within
  // a chunk is not user-visible, so reordering is allowed.
  Entity Archetype::SwapRemove(uint32_t _chunk_idx, uint32_t _row)
  {
    Chunk &chunk = *this->chunks_[_chunk_idx];
    assert(chunk.Count() > _row);

    // The removed row's component values are destroyed unconditionally;
    // no slot reuse without explicit re-construction.
    this->DestructRow(_chunk_idx, _row);

    uint32_t last = chunk.Count() - 1;
    if (_row == last)
    {
      // The removed row was already the tail — nothing to relocate.
      chunk.SetCount(last);
      chunk.Entities()[last] = kNullEntity;
      return kNullEntity;
    }

    // Move every column's last-row value into the now-empty victim
    // slot. Trivially relocatable types use a raw memcpy; others go
    // through the user-supplied move-construct + destruct pair.
    Entity moved = chunk.Entities()[last];
    chunk.Entities()[_row] = moved;

    for (size_t c = 0; c < this->infos_.size(); ++c)
    {
      auto *info = this->infos_[c];
      std::byte *col = chunk.ColumnBytes(this->col_offsets_[c]);
      void *dst = col + _row * info->size;
      void *src = col + last * info->size;

      if (info->trivially_relocatable)
      {
        std::memcpy(dst, src, info->size);
      }
      else
      {
        info->move(dst, src);
        info->destruct(src);
      }
    }

    chunk.Entities()[last] = kNullEntity;
    chunk.SetCount(last);
    return moved;
  }

  // Sum live-row counts across this archetype's chunks. O(num_chunks).
  size_t Archetype::TotalRows() const
  {
    size_t n = 0;
    for (auto &c : this->chunks_) n += c->Count();
    return n;
  }

  // Layout helper: how many 64-bit words of dirty bits each column
  // occupies, given `chunk_capacity_` rows. Used for indexing into the
  // packed bitset.
  size_t Archetype::DirtyWordsPerColumn() const
  {
    return (this->chunk_capacity_ + 63) / 64;
  }

  // Bulk-zero the dirty bits across every chunk in this archetype.
  // Called once per simulation step from `World::ClearChangeBits`,
  // typically after PostUpdate's `Commit`.
  void Archetype::ClearDirtyBits()
  {
    for (auto &chunk_ptr : this->chunks_)
    {
      Chunk &chunk = *chunk_ptr;
      if (chunk.DirtyBits() && chunk.DirtyWordCount())
      {
        std::memset(chunk.DirtyBits(), 0,
                    chunk.DirtyWordCount() * sizeof(uint64_t));
      }
    }
  }

  // Set the dirty bit for one (column, row) within a specific chunk.
  // The bitset is laid out as `[col0_words..., col1_words..., ...]`
  // where each block is `DirtyWordsPerColumn()` words long; row i
  // within a column lives at word `i / 64`, bit `i % 64`.
  void Archetype::MarkDirty(size_t _col_idx,
                            uint32_t _chunk_idx,
                            uint32_t _row)
  {
    Chunk &chunk = *this->chunks_[_chunk_idx];
    uint64_t *bits = chunk.DirtyBits();
    if (!bits) return;
    size_t wpc = this->DirtyWordsPerColumn();
    size_t base = _col_idx * wpc;
    size_t word = base + _row / 64;
    uint64_t mask = 1ull << (_row & 63u);
    bits[word] |= mask;
  }

  // Test the dirty bit for one (column, row). Mirror of MarkDirty.
  // Used by `EachChanged` to skip rows whose value didn't change this
  // step.
  bool Archetype::IsDirty(size_t _col_idx,
                          uint32_t _chunk_idx,
                          uint32_t _row) const
  {
    const Chunk &chunk = *this->chunks_[_chunk_idx];
    const uint64_t *bits = chunk.DirtyBits();
    if (!bits) return false;
    size_t wpc = this->DirtyWordsPerColumn();
    size_t base = _col_idx * wpc;
    size_t word = base + _row / 64;
    uint64_t mask = 1ull << (_row & 63u);
    return (bits[word] & mask) != 0;
  }
}
