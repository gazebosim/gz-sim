/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#include "gz/sim/ecs/detail/EntityIndex.hh"

#include <cassert>

namespace gz::sim::ecs
{
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
    return Entity(idx, slot.generation);
  }

  void EntityIndex::Free(Entity _e)
  {
    if (!this->IsAlive(_e))
      return;
    Slot &slot = this->slots_[_e.Index()];
    slot.alive = false;
    slot.archetype = kInvalidArchetypeId;
    // Bump generation so stale handles are detected.
    ++slot.generation;
    this->free_list_.push_back(_e.Index());
    assert(this->alive_count_ > 0);
    --this->alive_count_;
  }

  bool EntityIndex::IsAlive(Entity _e) const
  {
    const uint32_t idx = _e.Index();
    if (idx >= this->slots_.size())
      return false;
    const Slot &slot = this->slots_[idx];
    return slot.alive && slot.generation == _e.Generation();
  }

  EntityRecord EntityIndex::Get(Entity _e) const
  {
    const Slot &slot = this->slots_[_e.Index()];
    return {slot.archetype, slot.chunk_idx, slot.row};
  }

  void EntityIndex::Set(Entity _e, EntityRecord _rec)
  {
    Slot &slot = this->slots_[_e.Index()];
    slot.archetype = _rec.archetype;
    slot.chunk_idx = _rec.chunk_idx;
    slot.row = _rec.row;
  }

  void EntityIndex::PatchByIndex(uint32_t _slot_index, EntityRecord _rec)
  {
    Slot &slot = this->slots_[_slot_index];
    slot.archetype = _rec.archetype;
    slot.chunk_idx = _rec.chunk_idx;
    slot.row = _rec.row;
  }

  Entity EntityIndex::HandleFromSlot(uint32_t _slot_index) const
  {
    const Slot &slot = this->slots_[_slot_index];
    return Entity(_slot_index, slot.generation);
  }
}
