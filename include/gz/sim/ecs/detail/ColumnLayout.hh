/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_DETAIL_COLUMNLAYOUT_HH_
#define GZ_SIM_ECS_DETAIL_COLUMNLAYOUT_HH_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "gz/sim/ecs/ComponentTypeRegistry.hh"
#include "gz/sim/ecs/Entity.hh"

namespace gz::sim::ecs::detail
{
  /// \brief Minimum alignment per column. 16 keeps SSE loads aligned.
  inline constexpr size_t kMinColumnAlignment = 16;

  inline size_t AlignUp(size_t _v, size_t _a)
  {
    return (_v + _a - 1) & ~(_a - 1);
  }

  /// \brief Result of laying out columns within a chunk.
  struct ColumnLayout
  {
    /// \brief Byte offsets per column (one entry per input info).
    std::vector<size_t> offsets;

    /// \brief Number of rows per chunk the layout supports.
    size_t capacity{0};

    /// \brief Total bytes used per chunk at the chosen capacity (<= chunk size).
    size_t bytes_used{0};
  };

  /// \brief Compute a SoA column layout for an archetype.
  ///
  /// \param _infos ComponentTypeInfo* for every column in the archetype.
  ///   The Entity column is laid out first, at offset 0.
  /// \param _chunk_size Total chunk size in bytes.
  ///
  /// The algorithm binary-searches on row capacity: at each trial capacity N
  /// we lay out the columns and check if the total fits. This sidesteps the
  /// closed-form complexity of per-column alignment padding.
  inline ColumnLayout ComputeColumnLayout(
      const std::vector<const ComponentTypeInfo *> &_infos,
      size_t _chunk_size)
  {
    auto try_capacity = [&](size_t _cap) -> size_t
    {
      // Column 0 is always the Entity column.
      size_t offset = 0;
      offset += sizeof(Entity) * _cap;
      for (auto *info : _infos)
      {
        size_t align = info->alignment < kMinColumnAlignment
            ? kMinColumnAlignment
            : info->alignment;
        offset = AlignUp(offset, align);
        offset += info->size * _cap;
      }
      return offset;
    };

    // Upper bound on capacity: assume zero padding.
    size_t row_bytes = sizeof(Entity);
    for (auto *info : _infos)
      row_bytes += info->size;
    size_t lo = 1;
    size_t hi = row_bytes == 0 ? _chunk_size : _chunk_size / row_bytes;
    if (hi < 1) hi = 1;

    // Binary search: largest N such that try_capacity(N) <= _chunk_size.
    size_t best = 0;
    while (lo <= hi)
    {
      size_t mid = lo + (hi - lo) / 2;
      if (try_capacity(mid) <= _chunk_size)
      {
        best = mid;
        lo = mid + 1;
      }
      else
      {
        if (mid == 0) break;
        hi = mid - 1;
      }
    }

    ColumnLayout layout;
    layout.capacity = best;
    if (best == 0)
    {
      return layout;
    }

    size_t offset = sizeof(Entity) * best;
    layout.offsets.reserve(_infos.size() + 1);
    // Entity column offset (always 0), we do not emit it into offsets — users
    // index offsets[i] for component i.
    for (auto *info : _infos)
    {
      size_t align = info->alignment < kMinColumnAlignment
          ? kMinColumnAlignment
          : info->alignment;
      offset = AlignUp(offset, align);
      layout.offsets.push_back(offset);
      offset += info->size * best;
    }
    layout.bytes_used = offset;
    return layout;
  }
}

#endif
