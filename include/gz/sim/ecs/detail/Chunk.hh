/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_CHUNK_HH_
#define GZ_SIM_ECS_CHUNK_HH_

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>

#include "gz/sim/ecs/Entity.hh"

namespace gz::sim::ecs
{
  /// \brief Aligned raw-byte block owned by an Archetype.
  ///
  /// A Chunk's layout is determined by its owning Archetype (see
  /// detail::ComputeColumnLayout). The Chunk only stores the raw bytes, the
  /// current row count, and the number of dirty-bit words per column.
  class Chunk
  {
    public: static constexpr size_t kDefaultSize = 16u * 1024u;
    public: static constexpr size_t kAlignment = 64u;

    public: explicit Chunk(size_t _size = kDefaultSize, size_t _capacity = 0);
    public: ~Chunk();

    public: Chunk(const Chunk &) = delete;
    public: Chunk &operator=(const Chunk &) = delete;

    /// \brief Pointer to column 0 (entity handles).
    public: Entity *Entities()
    {
      return reinterpret_cast<Entity *>(this->storage_);
    }
    public: const Entity *Entities() const
    {
      return reinterpret_cast<const Entity *>(this->storage_);
    }

    /// \brief Raw pointer to the beginning of a column. Archetype holds the
    /// per-column byte offsets.
    public: std::byte *ColumnBytes(size_t _offset)
    {
      return this->storage_ + _offset;
    }
    public: const std::byte *ColumnBytes(size_t _offset) const
    {
      return this->storage_ + _offset;
    }

    public: template <class T>
    T *Column(size_t _offset)
    {
      return reinterpret_cast<T *>(this->storage_ + _offset);
    }

    public: template <class T>
    const T *Column(size_t _offset) const
    {
      return reinterpret_cast<const T *>(this->storage_ + _offset);
    }

    public: uint32_t Count() const { return this->count_; }
    public: uint32_t Capacity() const { return this->capacity_; }
    public: size_t   Size()  const { return this->size_; }

    public: bool Full() const { return this->count_ >= this->capacity_; }
    public: bool Empty() const { return this->count_ == 0; }

    /// \brief Reserve (but do not construct into) the next row. Caller is
    /// responsible for placement-constructing every column at the returned
    /// row index. Returns kInvalidRow (UINT32_MAX) if the chunk is full.
    public: uint32_t AcquireRow();

    /// \brief Release the last row (destructive — decrements count).
    /// Caller is responsible for running destructors of the columns.
    public: void ReleaseLastRow();

    /// \brief Force the count to an exact value (for row-swap operations).
    public: void SetCount(uint32_t _count) { this->count_ = _count; }

    /// \brief Dirty bitsets live in a separate heap allocation sized by
    /// (num_columns * words_per_column). Maintained by Archetype.
    public: uint64_t *DirtyBits() { return this->dirty_bits_.get(); }
    public: const uint64_t *DirtyBits() const { return this->dirty_bits_.get(); }
    public: void AllocDirtyBits(size_t _words);
    public: size_t DirtyWordCount() const { return this->dirty_word_count_; }

    public: static constexpr uint32_t kInvalidRow = 0xFFFFFFFFu;

    private: std::byte *storage_{nullptr};
    private: size_t    size_{0};
    private: uint32_t  count_{0};
    private: uint32_t  capacity_{0};
    private: std::unique_ptr<uint64_t[]> dirty_bits_;
    private: size_t    dirty_word_count_{0};
  };
}

#endif
