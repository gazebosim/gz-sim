/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_QUERY_HH_
#define GZ_SIM_ECS_QUERY_HH_

#include <cstdint>
#include <vector>

#include "gz/sim/ecs/ComponentTypeRegistry.hh"

namespace gz::sim::ecs
{
  class World;

  /// \brief Cached matching-archetype list for a (required, excluded) tuple
  /// of component types. Rematches only when the archetype-graph version
  /// changes.
  class Query
  {
    public: Query() = default;

    public: Query(std::vector<ComponentTypeId> _required,
                  std::vector<ComponentTypeId> _excluded = {});

    public: const std::vector<ComponentTypeId> &Required() const
    { return this->required_; }
    public: const std::vector<ComponentTypeId> &Excluded() const
    { return this->excluded_; }

    public: const std::vector<uint32_t> &Matching() const
    { return this->matching_; }

    /// \brief Refresh the matching archetype list if the world's graph
    /// version has changed. Called automatically by World::Each/EachParallel.
    public: void RefreshIfStale(const World &_world);

    /// \brief Force a re-match.
    public: void Refresh(const World &_world);

    private: std::vector<ComponentTypeId> required_;
    private: std::vector<ComponentTypeId> excluded_;
    private: std::vector<uint32_t>        matching_;
    private: uint64_t                     cache_version_{
        static_cast<uint64_t>(-1)};
  };
}

#endif
