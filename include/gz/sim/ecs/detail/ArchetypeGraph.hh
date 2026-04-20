/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_ARCHETYPEGRAPH_HH_
#define GZ_SIM_ECS_ARCHETYPEGRAPH_HH_

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "gz/sim/ecs/ComponentTypeRegistry.hh"
#include "gz/sim/ecs/detail/Archetype.hh"
#include "gz/sim/ecs/detail/EntityIndex.hh"

namespace gz::sim::ecs
{
  /// \brief Registry of Archetypes keyed by their sorted type vector, plus
  /// cached transition edges (add T / remove T) per archetype.
  class ArchetypeGraph
  {
    public: ArchetypeGraph();
    public: ~ArchetypeGraph();

    public: Archetype &Get(ArchetypeId _id);
    public: const Archetype &Get(ArchetypeId _id) const;

    public: size_t Size() const { return this->archetypes_.size(); }
    public: uint64_t Version() const { return this->version_; }

    /// \brief Look up (or create) the archetype for a sorted type vector.
    public: ArchetypeId GetOrCreate(
        const std::vector<ComponentTypeId> &_types);

    /// \brief Get (or create) the archetype obtained by adding `_type` to
    /// the archetype identified by `_src`.
    public: ArchetypeId AddEdge(ArchetypeId _src, ComponentTypeId _type);

    /// \brief Get (or create) the archetype obtained by removing `_type`.
    /// Returns kInvalidArchetypeId if _src does not contain _type.
    public: ArchetypeId RemoveEdge(ArchetypeId _src, ComponentTypeId _type);

    /// \brief Iteration helper — enumerate all archetypes in insertion order.
    public: template <class Fn>
    void ForEach(Fn &&_fn)
    {
      for (auto &ap : this->archetypes_) _fn(*ap);
    }

    public: template <class Fn>
    void ForEach(Fn &&_fn) const
    {
      for (auto &ap : this->archetypes_) _fn(*ap);
    }

    private: struct Key
    {
      std::vector<ComponentTypeId> types;
      bool operator==(const Key &_o) const { return this->types == _o.types; }
    };

    private: struct KeyHash
    {
      size_t operator()(const Key &_k) const
      {
        size_t h = 1469598103934665603ull;
        for (auto t : _k.types)
        {
          h ^= t;
          h *= 1099511628211ull;
        }
        return h;
      }
    };

    // Edge cache per archetype, keyed by type id.
    private: struct EdgeTable
    {
      std::unordered_map<ComponentTypeId, ArchetypeId> add;
      std::unordered_map<ComponentTypeId, ArchetypeId> remove;
    };

    private: ArchetypeId AllocateArchetype(
        const std::vector<ComponentTypeId> &_types);

    private: std::vector<std::unique_ptr<Archetype>> archetypes_;
    private: std::vector<EdgeTable>                  edges_;
    private: std::unordered_map<Key, ArchetypeId, KeyHash> by_types_;
    private: uint64_t                                version_{0};
  };
}

#endif
