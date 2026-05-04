/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// ComponentTypeRegistry — process-wide table mapping `ComponentTypeId`
// to its `ComponentTypeInfo` (size, alignment, lifecycle hooks,
// trivially-relocatable flag, debug name).
//
// Registration is append-only and idempotent: re-registering the same
// id returns the existing entry. Lookups are linear over a small
// table (in practice a few dozen to a few hundred entries) — the
// constant factor is small enough that a hash map's overhead would
// be net loss.
//
// Thread safety: a single mutex guards both insertion and lookup.
// Lookup is a cold-ish path called once per archetype creation
// (via `ArchetypeGraph::AllocateArchetype`); the per-row inner
// loops cache the `ComponentTypeInfo *` on the archetype itself, so
// they never touch this registry.
#include "gz/sim/ecs/ComponentTypeRegistry.hh"

namespace gz::sim::ecs
{
  // Meyers singleton — thread-safe initialization in C++17.
  ComponentTypeRegistry &ComponentTypeRegistry::Instance()
  {
    static ComponentTypeRegistry inst;
    return inst;
  }

  // Look up a registered type by id. Returns nullptr if the id has
  // never been registered (caller decides whether that's an error).
  const ComponentTypeInfo *ComponentTypeRegistry::Get(
      ComponentTypeId _id) const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    for (auto *e : this->entries_)
    {
      if (e->id == _id)
        return e;
    }
    return nullptr;
  }

  // Install (or look up) a type by id. The first caller for a given
  // id wins — subsequent calls return the existing entry without
  // overwriting. Returned pointer is stable for the process lifetime
  // (the entries are heap-allocated and never freed in production).
  const ComponentTypeInfo *ComponentTypeRegistry::Install(
      ComponentTypeInfo &&_info)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    for (auto *e : this->entries_)
    {
      if (e->id == _info.id)
        return e;
    }
    auto *heap = new ComponentTypeInfo(std::move(_info));
    this->entries_.push_back(heap);
    return heap;
  }

  size_t ComponentTypeRegistry::Size() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->entries_.size();
  }

  // Test-only escape hatch. The registry is process-wide and
  // append-only by design, but unit tests benefit from being able
  // to start each case from a clean slate. Production code must
  // never call this — any live World holds `ComponentTypeInfo *`
  // pointers that this would dangle.
  void ComponentTypeRegistry::ClearForTesting()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    for (auto *e : this->entries_)
      delete e;
    this->entries_.clear();
  }
}
