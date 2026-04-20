/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#include "gz/sim/ecs/detail/Archetype.hh"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <stdexcept>

#include "gz/sim/ecs/detail/ColumnLayout.hh"

namespace gz::sim::ecs
{
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
    // Types must be sorted for archetype equality.
    for (size_t i = 1; i < this->types_.size(); ++i)
      assert(this->types_[i - 1] < this->types_[i]);

    auto layout = detail::ComputeColumnLayout(this->infos_, _chunk_size);
    if (layout.capacity == 0)
    {
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

  Archetype::~Archetype()
  {
    // Destruct everything still live in each chunk.
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

  size_t Archetype::ColumnIndexOf(ComponentTypeId _id) const
  {
    auto it = std::lower_bound(
        this->types_.begin(), this->types_.end(), _id);
    if (it == this->types_.end() || *it != _id)
      return static_cast<size_t>(-1);
    return static_cast<size_t>(it - this->types_.begin());
  }

  bool Archetype::Contains(ComponentTypeId _id) const
  {
    return this->ColumnIndexOf(_id) != static_cast<size_t>(-1);
  }

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

    // Spill to a fresh chunk.
    auto chunk = std::make_unique<Chunk>(
        this->chunk_size_, this->chunk_capacity_);

    // Allocate dirty bits: one bit per (row, column).
    size_t words_per_col = (this->chunk_capacity_ + 63) / 64;
    chunk->AllocDirtyBits(words_per_col * this->infos_.size());

    uint32_t row = chunk->AcquireRow();
    this->chunks_.push_back(std::move(chunk));
    return {static_cast<uint32_t>(this->chunks_.size() - 1), row};
  }

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

  Entity Archetype::SwapRemove(uint32_t _chunk_idx, uint32_t _row)
  {
    Chunk &chunk = *this->chunks_[_chunk_idx];
    assert(chunk.Count() > _row);

    // Destruct the removed row first.
    this->DestructRow(_chunk_idx, _row);

    uint32_t last = chunk.Count() - 1;
    if (_row == last)
    {
      // Just pop.
      chunk.SetCount(last);
      // Zero entity slot defensively.
      chunk.Entities()[last] = kNullEntity;
      return kNullEntity;
    }

    // Move the last row's columns into the victim's place. Use memcpy for
    // trivially relocatable types, move-construct otherwise.
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

  size_t Archetype::TotalRows() const
  {
    size_t n = 0;
    for (auto &c : this->chunks_) n += c->Count();
    return n;
  }

  size_t Archetype::DirtyWordsPerColumn() const
  {
    return (this->chunk_capacity_ + 63) / 64;
  }

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
