/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#include "gz/sim/ecs/detail/Chunk.hh"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <new>

namespace gz::sim::ecs
{
  namespace
  {
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

    void AlignedFree(std::byte *_p)
    {
#if defined(_WIN32)
      _aligned_free(_p);
#else
      std::free(_p);
#endif
    }
  }

  Chunk::Chunk(size_t _size, size_t _capacity)
    : size_(_size), capacity_(static_cast<uint32_t>(_capacity))
  {
    this->storage_ = AlignedAlloc(kAlignment, _size);
    if (!this->storage_)
      throw std::bad_alloc();
    // Zero the storage so Entity column rows are well-defined sentinels
    // before they are populated.
    std::memset(this->storage_, 0, _size);
  }

  Chunk::~Chunk()
  {
    AlignedFree(this->storage_);
    this->storage_ = nullptr;
  }

  uint32_t Chunk::AcquireRow()
  {
    if (this->count_ >= this->capacity_)
      return kInvalidRow;
    return this->count_++;
  }

  void Chunk::ReleaseLastRow()
  {
    assert(this->count_ > 0);
    --this->count_;
  }

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
