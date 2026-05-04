/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// CommandBuffer — per-phase / per-thread record of deferred mutations.
//
// When `World` is in a phase (between `BeginPhase()` and `Commit()`),
// component creates / destroys / writes are not applied immediately.
// They are encoded as `Command` records and the component payload bytes
// are appended into a side blob. `Commit()` walks each thread's buffer
// in deterministic order, replays the commands, and frees the blob.
//
// Lock-free on the hot path: each worker thread owns its own
// `CommandBuffer`, vended by `World::LocalBuffer()`.
#include "gz/sim/ecs/CommandBuffer.hh"

#include <cstring>

namespace gz::sim::ecs
{
  // Append `_n` bytes from `_src` into the blob arena and return the
  // byte offset of the start of the copy. The offset (not a pointer)
  // is what the `Command` record stores, because `blob_` may grow and
  // reallocate on subsequent appends.
  uint32_t CommandBuffer::AppendPayload(const void *_src, size_t _n)
  {
    const uint32_t off = static_cast<uint32_t>(this->blob_.size());
    this->blob_.resize(this->blob_.size() + _n);
    std::memcpy(this->blob_.data() + off, _src, _n);
    return off;
  }
}
