/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// EntityIndex — maps `Entity` handles to their `(archetype, chunk, row)`
// location in chunk storage.
//
// An `Entity` is a 64-bit value: low 32 bits hold an index into the
// slot vector below, high 32 bits hold a generation counter. The
// generation increments every time a slot is freed, so a stale handle
// (one whose entity was destroyed and the slot reused) fails the
// generation check in `IsAlive` and is rejected by every read path.
//
// Slots are reused via a free list to keep the index dense. The
// generation counter is what makes that reuse safe — without it,
// reading from a stale handle would silently return another entity's
// data.
#include "gz/sim/ecs/detail/EntityIndex.hh"

#include <cassert>

namespace gz::sim::ecs
{
  // Hand out a fresh `Entity` handle. Reuses a slot from the free
  // list if any, otherwise grows the slot vector by one. The new
  // entity isn't placed into any archetype yet — `Set()` from the
  // World does that immediately after, with the empty archetype's
  // `(0, 0)` chunk-row pair.
  Entity EntityIndex::Allocate()
  {
    uint32_t idx;
    if (!this->free_list_.empty())
    {
      idx = this->free_list_.back();
      this->free_list_.pop_back();
    }
    else
    {
      idx = static_cast<uint32_t>(this->slots_.size());
      this->slots_.emplace_back();
    }

    Slot &slot = this->slots_[idx];
    slot.alive = true;
    slot.archetype = kInvalidArchetypeId;
    slot.chunk_idx = 0;
    slot.row = 0;
    ++this->alive_count_;
    // The slot's generation carries over from any prior occupant —
    // the `++` happens at Free() time, not here, so this handle
    // already has the post-bump generation.
    return Entity(idx, slot.generation);
  }

  // Mark `_e`'s slot as dead and push it onto the free list. Bumps
  // the generation so any outstanding handles to `_e` will fail
  // `IsAlive`. Idempotent on already-dead handles.
  void EntityIndex::Free(Entity _e)
  {
    if (!this->IsAlive(_e))
      return;
    Slot &slot = this->slots_[_e.Index()];
    slot.alive = false;
    slot.archetype = kInvalidArchetypeId;
    ++slot.generation;
    this->free_list_.push_back(_e.Index());
    assert(this->alive_count_ > 0);
    --this->alive_count_;
  }

  // Validate `_e` against the stored slot. False if:
  //   * The index is past the end of the slot vector (never used).
  //   * The slot is currently dead (entity was destroyed).
  //   * The generations differ (entity was destroyed and the slot
  //     was reused — `_e` is a stale handle).
  bool EntityIndex::IsAlive(Entity _e) const
  {
    const uint32_t idx = _e.Index();
    if (idx >= this->slots_.size())
      return false;
    const Slot &slot = this->slots_[idx];
    return slot.alive && slot.generation == _e.Generation();
  }

  // Look up the chunk/row coordinates for a known-alive entity. The
  // caller is responsible for having already checked `IsAlive`;
  // calling Get on a dead handle returns the stale slot data.
  EntityRecord EntityIndex::Get(Entity _e) const
  {
    const Slot &slot = this->slots_[_e.Index()];
    return {slot.archetype, slot.chunk_idx, slot.row};
  }

  // Update the chunk/row coordinates for `_e`. Called after every
  // archetype transition — the World moves the row, then patches
  // the index here so subsequent lookups see the new location.
  void EntityIndex::Set(Entity _e, EntityRecord _rec)
  {
    Slot &slot = this->slots_[_e.Index()];
    slot.archetype = _rec.archetype;
    slot.chunk_idx = _rec.chunk_idx;
    slot.row = _rec.row;
  }

  // Slot-index variant of `Set` used by the swap-with-last machinery
  // in `Archetype::SwapRemove`. After SwapRemove returns the entity
  // that got relocated (the chunk's previous last row), the World
  // patches that entity's slot here using the slot index extracted
  // from `Entity::Index()` — without re-deriving the entity handle.
  void EntityIndex::PatchByIndex(uint32_t _slot_index, EntityRecord _rec)
  {
    Slot &slot = this->slots_[_slot_index];
    slot.archetype = _rec.archetype;
    slot.chunk_idx = _rec.chunk_idx;
    slot.row = _rec.row;
  }

  // Reconstruct the current `Entity` handle for a slot index. Used by
  // tests and by debug assertions; the generation in the returned
  // handle always matches the slot's current generation, so the
  // result is "alive" by construction.
  Entity EntityIndex::HandleFromSlot(uint32_t _slot_index) const
  {
    const Slot &slot = this->slots_[_slot_index];
    return Entity(_slot_index, slot.generation);
  }
}
