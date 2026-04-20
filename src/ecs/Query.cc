/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#include "gz/sim/ecs/Query.hh"

#include <algorithm>

#include "gz/sim/ecs/World.hh"
#include "gz/sim/ecs/detail/Archetype.hh"
#include "gz/sim/ecs/detail/ArchetypeGraph.hh"

namespace gz::sim::ecs
{
  Query::Query(std::vector<ComponentTypeId> _required,
               std::vector<ComponentTypeId> _excluded)
    : required_(std::move(_required)), excluded_(std::move(_excluded))
  {
    std::sort(this->required_.begin(), this->required_.end());
    std::sort(this->excluded_.begin(), this->excluded_.end());
  }

  void Query::RefreshIfStale(const World &_world)
  {
    if (this->cache_version_ != _world.Graph().Version())
      this->Refresh(_world);
  }

  void Query::Refresh(const World &_world)
  {
    this->matching_.clear();
    const auto &graph = _world.Graph();
    graph.ForEach([this](const Archetype &_a)
    {
      const auto &types = _a.Types();
      // All required must be in types.
      for (auto t : this->required_)
      {
        if (!std::binary_search(types.begin(), types.end(), t))
          return;
      }
      // None of excluded may be in types.
      for (auto t : this->excluded_)
      {
        if (std::binary_search(types.begin(), types.end(), t))
          return;
      }
      this->matching_.push_back(_a.Id());
    });
    this->cache_version_ = _world.Graph().Version();
  }
}
