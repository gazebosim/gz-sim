/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#include "gz/sim/ecs/CommandBuffer.hh"

#include <cstring>

namespace gz::sim::ecs
{
  uint32_t CommandBuffer::AppendPayload(const void *_src, size_t _n)
  {
    const uint32_t off = static_cast<uint32_t>(this->blob_.size());
    this->blob_.resize(this->blob_.size() + _n);
    std::memcpy(this->blob_.data() + off, _src, _n);
    return off;
  }
}
