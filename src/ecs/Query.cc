/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// Query — compiled match list of archetypes that satisfy a
// required-types / excluded-types filter.
//
// A Query is typically constructed once per `Each<T...>` callsite and
// cached as a static local. The first call against a fresh world
// scans every archetype and records the matching IDs; subsequent
// calls reuse the cached list. The cache invalidates only when a new
// archetype is created (the graph's monotonic version counter
// advances).
//
// Removal of archetypes isn't supported, so cached IDs never go
// stale — they only need to be extended when the graph grows.
#include "gz/sim/ecs/Query.hh"

#include <algorithm>

#include "gz/sim/ecs/World.hh"
#include "gz/sim/ecs/detail/Archetype.hh"
#include "gz/sim/ecs/detail/ArchetypeGraph.hh"

namespace gz::sim::ecs
{
  // Sort both filter vectors so the per-archetype matching loop in
  // `Refresh` can use binary search against the archetype's
  // (already-sorted) type set.
  Query::Query(std::vector<ComponentTypeId> _required,
               std::vector<ComponentTypeId> _excluded)
    : required_(std::move(_required)), excluded_(std::move(_excluded))
  {
    std::sort(this->required_.begin(), this->required_.end());
    std::sort(this->excluded_.begin(), this->excluded_.end());
  }

  // Cheap version check on the hot path. The world's graph bumps
  // its version counter every time a new archetype is created;
  // anything else (entity create/destroy, component add/remove on
  // existing archetypes) leaves it untouched. So most steps this is
  // a single integer compare.
  void Query::RefreshIfStale(const World &_world)
  {
    if (this->cache_version_ != _world.Graph().Version())
      this->Refresh(_world);
  }

  // Full re-scan of the archetype graph. For each archetype, it
  // qualifies if every required type is present and no excluded
  // type is present. Both checks use `binary_search` against the
  // archetype's sorted type set.
  void Query::Refresh(const World &_world)
  {
    this->matching_.clear();
    const auto &graph = _world.Graph();
    graph.ForEach([this](const Archetype &_a)
    {
      const auto &types = _a.Types();
      for (auto t : this->required_)
      {
        if (!std::binary_search(types.begin(), types.end(), t))
          return;
      }
      for (auto t : this->excluded_)
      {
        if (std::binary_search(types.begin(), types.end(), t))
          return;
      }
      this->matching_.push_back(_a.Id());
    });
    // Snapshot the version we built against; subsequent
    // `RefreshIfStale` calls compare to this.
    this->cache_version_ = _world.Graph().Version();
  }
}
