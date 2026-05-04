/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// Chunk — fixed-size aligned storage block owned by an Archetype.
//
// A Chunk holds up to `capacity_` rows of struct-of-arrays component
// data plus an Entity column. Per-column byte offsets within the chunk
// live on the owning Archetype (precomputed once at construction);
// this file only manages the raw storage, the row counter, and the
// optional dirty-bit bitset.
//
// Aligned allocation is needed so each column starts on at least a
// 16-byte boundary regardless of where the chunk lands on the heap —
// this is what enables vectorized loads/stores in the Each<T...> hot
// loop.
#include "gz/sim/ecs/detail/Chunk.hh"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <new>

namespace gz::sim::ecs
{
  namespace
  {
    // Cross-platform aligned heap allocator. The standard
    // `std::aligned_alloc` exists, but rounds the size up to a
    // multiple of the alignment, which complicates capacity math. The
    // platform primitives don't have that quirk.
    std::byte *AlignedAlloc(size_t _align, size_t _size)
    {
#if defined(_WIN32)
      return static_cast<std::byte *>(_aligned_malloc(_size, _align));
#else
      void *ptr = nullptr;
      if (posix_memalign(&ptr, _align, _size) != 0)
        return nullptr;
      return static_cast<std::byte *>(ptr);
#endif
    }

    // Matches AlignedAlloc — Windows requires `_aligned_free`, POSIX
    // accepts plain `free`.
    void AlignedFree(std::byte *_p)
    {
#if defined(_WIN32)
      _aligned_free(_p);
#else
      std::free(_p);
#endif
    }
  }

  // Construct a chunk of `_size` bytes able to hold `_capacity` rows.
  // Capacity is computed by the owning Archetype from the chunk size
  // and the per-row byte cost of its column set; we just trust the
  // value here and stash it for AcquireRow() bounds checks.
  //
  // The storage is zero-initialized so that the Entity column reads
  // back as `kNullEntity` for unused rows — useful for asserts and
  // for the "zero entity slot defensively" idiom in SwapRemove().
  Chunk::Chunk(size_t _size, size_t _capacity)
    : size_(_size), capacity_(static_cast<uint32_t>(_capacity))
  {
    this->storage_ = AlignedAlloc(kAlignment, _size);
    if (!this->storage_)
      throw std::bad_alloc();
    std::memset(this->storage_, 0, _size);
  }

  // The Archetype destructor walks every live row and runs each
  // column's destructor before this destructor releases the storage,
  // so we don't need to do anything here except free the buffer.
  Chunk::~Chunk()
  {
    AlignedFree(this->storage_);
    this->storage_ = nullptr;
  }

  // Reserve the next free row slot, returning its row index. Callers
  // must check for `kInvalidRow` (chunk is full) before writing.
  // Linear bump-allocator: rows are handed out in order 0, 1, 2, …
  // and only released by SwapRemove() / ReleaseLastRow().
  uint32_t Chunk::AcquireRow()
  {
    if (this->count_ >= this->capacity_)
      return kInvalidRow;
    return this->count_++;
  }

  // Pop the most recently acquired row. Used by the swap-remove
  // path in Archetype after copying the last row over a deleted one.
  void Chunk::ReleaseLastRow()
  {
    assert(this->count_ > 0);
    --this->count_;
  }

  // Allocate the per-chunk dirty-bit bitset. The Archetype computes
  // `_words` as `(chunk_capacity + 63) / 64 * num_columns` so that
  // every (column, row) pair has its own bit. Passing `_words == 0`
  // releases the bitset entirely (used by archetypes with no
  // change-tracked columns, though in practice every archetype uses
  // them).
  void Chunk::AllocDirtyBits(size_t _words)
  {
    this->dirty_word_count_ = _words;
    if (_words == 0)
    {
      this->dirty_bits_.reset();
      return;
    }
    this->dirty_bits_ = std::make_unique<uint64_t[]>(_words);
    std::memset(this->dirty_bits_.get(), 0, _words * sizeof(uint64_t));
  }
}
