/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_ENTITYINDEX_HH_
#define GZ_SIM_ECS_ENTITYINDEX_HH_

#include <cstdint>
#include <limits>
#include <vector>

#include "gz/sim/ecs/Entity.hh"

namespace gz::sim::ecs
{
  /// \brief Internal dense ID for an archetype.
  using ArchetypeId = uint32_t;
  inline constexpr ArchetypeId kInvalidArchetypeId =
      std::numeric_limits<ArchetypeId>::max();

  /// \brief Canonical location of an entity's component data.
  struct EntityRecord
  {
    ArchetypeId archetype{kInvalidArchetypeId};
    uint32_t    chunk_idx{0};
    uint32_t    row{0};
  };

  /// \brief Dense, generation-checked slot table for Entity handles.
  /// Provides O(1) allocation, lookup, and erase (with a single swap-patch
  /// for the victim of a row swap-with-last).
  class EntityIndex
  {
    public: struct Slot
    {
      ArchetypeId archetype{kInvalidArchetypeId};
      uint32_t    chunk_idx{0};
      uint32_t    row{0};
      uint32_t    generation{0};
      bool        alive{false};
    };

    /// \brief Allocate a new Entity. Reuses a slot from the free list if one
    /// is available; otherwise grows the slots vector.
    public: Entity Allocate();

    /// \brief Mark a slot dead and push it onto the free list. The slot's
    /// generation is bumped so subsequent dereference of old handles fails.
    public: void Free(Entity _e);

    /// \brief True if the handle's generation still matches its slot.
    public: bool IsAlive(Entity _e) const;

    /// \brief Fetch the record for a live entity. Undefined if !IsAlive.
    public: EntityRecord Get(Entity _e) const;

    /// \brief Update the record for a live entity.
    public: void Set(Entity _e, EntityRecord _rec);

    /// \brief Directly patch a record by slot index — used when a row swap
    /// inside a chunk relocates a victim entity.
    public: void PatchByIndex(uint32_t _slot_index, EntityRecord _rec);

    /// \brief Entity handle from a live slot index (for reverse lookup inside
    /// chunks via the entity column).
    public: Entity HandleFromSlot(uint32_t _slot_index) const;

    /// \brief Number of currently alive entities.
    public: size_t AliveCount() const { return alive_count_; }

    /// \brief Capacity of the slot table.
    public: size_t Capacity() const { return slots_.size(); }

    /// \brief Access a slot by index (read-only, for debug / iteration).
    public: const Slot &SlotAt(uint32_t _i) const { return slots_[_i]; }

    private: std::vector<Slot>     slots_;
    private: std::vector<uint32_t> free_list_;
    private: size_t                alive_count_{0};
  };
}

#endif
