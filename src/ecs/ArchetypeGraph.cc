/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// ArchetypeGraph — central registry of archetypes and the cached edges
// between them.
//
// "Edges" describe the result of a single-component mutation: the
// `add` edge from archetype A under type T points at the archetype
// you land in after adding type T to an entity in A. The `remove`
// edge is the inverse. Edges are populated lazily on first use and
// cached forever, so a hot-path "tag this entity" pattern is O(1)
// after warm-up.
//
// Lookups by type-set go through `by_types_`, a hash map keyed by the
// sorted type vector. The graph is monotonic in 0a — archetypes are
// never removed, so vector indices and hash entries are stable for
// the lifetime of the World.
//
// Subtle invariant: every helper that may call `GetOrCreate` must
// re-fetch any `edges_` / `archetypes_` references after the call.
// Allocation can grow those vectors and invalidate any pointers /
// references taken before. The `AddEdge` / `RemoveEdge` bodies below
// follow that rule deliberately.
#include "gz/sim/ecs/detail/ArchetypeGraph.hh"

#include <algorithm>
#include <cassert>
#include <stdexcept>

namespace gz::sim::ecs
{
  // Pre-allocate the empty archetype (ArchetypeId 0). Every entity
  // is born here via `World::CreateEmpty`; subsequent `Add` calls
  // walk the graph from this root.
  ArchetypeGraph::ArchetypeGraph()
  {
    this->AllocateArchetype({});
  }

  ArchetypeGraph::~ArchetypeGraph() = default;

  // Direct ID lookup. Archetypes are never removed, so the index is
  // stable for the World's lifetime.
  Archetype &ArchetypeGraph::Get(ArchetypeId _id)
  {
    return *this->archetypes_[_id];
  }

  const Archetype &ArchetypeGraph::Get(ArchetypeId _id) const
  {
    return *this->archetypes_[_id];
  }

  // Construct a brand-new archetype for `_types`, register it in the
  // hash map, and bump the version counter so dependent `Query`
  // caches refresh on next use. The caller is responsible for having
  // already checked that no archetype with this exact type set
  // exists — `GetOrCreate` is the public-facing wrapper that does
  // the check.
  //
  // Throws if any type in `_types` isn't registered with the
  // ComponentTypeRegistry — the registry is the only source of
  // ComponentTypeInfo, which the Archetype constructor needs for
  // column layout.
  ArchetypeId ArchetypeGraph::AllocateArchetype(
      const std::vector<ComponentTypeId> &_types)
  {
    std::vector<const ComponentTypeInfo *> infos;
    infos.reserve(_types.size());
    auto &reg = ComponentTypeRegistry::Instance();
    for (auto t : _types)
    {
      auto *info = reg.Get(t);
      if (!info)
      {
        throw std::runtime_error(
            "ArchetypeGraph: component type not registered");
      }
      infos.push_back(info);
    }

    ArchetypeId id = static_cast<ArchetypeId>(this->archetypes_.size());
    this->archetypes_.push_back(
        std::make_unique<Archetype>(id, _types, std::move(infos)));
    this->edges_.emplace_back();
    Key k{_types};
    this->by_types_.emplace(std::move(k), id);
    ++this->version_;
    return id;
  }

  // Look up the archetype for `_types`, creating it if it doesn't
  // exist yet. `_types` must be sorted and unique — the assert below
  // catches buggy callers in debug builds. (The hash key relies on the
  // sort order; passing the same set in different orders would create
  // duplicate archetypes.)
  ArchetypeId ArchetypeGraph::GetOrCreate(
      const std::vector<ComponentTypeId> &_types)
  {
#ifndef NDEBUG
    for (size_t i = 1; i < _types.size(); ++i)
      assert(_types[i - 1] < _types[i] &&
             "ArchetypeGraph: types must be sorted and unique");
#endif
    Key k{_types};
    auto it = this->by_types_.find(k);
    if (it != this->by_types_.end())
      return it->second;
    return this->AllocateArchetype(_types);
  }

  // Compute the destination archetype obtained by adding `_type` to
  // `_src`'s type set, and cache the edge for future lookups.
  // Returns `_src` itself if the type is already in the set
  // (idempotent add).
  ArchetypeId ArchetypeGraph::AddEdge(ArchetypeId _src, ComponentTypeId _type)
  {
    // Cache hit — this is the common case after warm-up.
    {
      auto it = this->edges_[_src].add.find(_type);
      if (it != this->edges_[_src].add.end())
        return it->second;
    }

    // Take a value-copy of the source type vector. `GetOrCreate`
    // below may allocate a new archetype, which can grow `archetypes_`
    // and `edges_` and invalidate any reference into them. Working
    // off the copy is safe across the call.
    std::vector<ComponentTypeId> src_types = this->archetypes_[_src]->Types();
    std::vector<ComponentTypeId> dst;
    dst.reserve(src_types.size() + 1);

    // Splice `_type` into the sorted position. If we find it already
    // present, the add is a no-op — record the self-edge and return.
    bool inserted = false;
    for (auto t : src_types)
    {
      if (!inserted && _type < t)
      {
        dst.push_back(_type);
        inserted = true;
      }
      else if (!inserted && _type == t)
      {
        this->edges_[_src].add.emplace(_type, _src);
        return _src;
      }
      dst.push_back(t);
    }
    if (!inserted) dst.push_back(_type);

    ArchetypeId dst_id = this->GetOrCreate(dst);
    // Cache both directions: src --add(_type)--> dst, dst --remove(_type)--> src.
    this->edges_[_src].add.emplace(_type, dst_id);
    this->edges_[dst_id].remove.emplace(_type, _src);
    return dst_id;
  }

  // Compute the destination archetype obtained by removing `_type`
  // from `_src`'s type set, and cache the edge. Returns
  // `kInvalidArchetypeId` if `_type` isn't actually in `_src` (the
  // remove is meaningless and the caller treats this as a no-op).
  ArchetypeId ArchetypeGraph::RemoveEdge(
      ArchetypeId _src, ComponentTypeId _type)
  {
    // Cache hit.
    {
      auto it = this->edges_[_src].remove.find(_type);
      if (it != this->edges_[_src].remove.end())
        return it->second;
    }

    // Same value-copy discipline as AddEdge above.
    std::vector<ComponentTypeId> src_types = this->archetypes_[_src]->Types();
    auto sit = std::lower_bound(src_types.begin(), src_types.end(), _type);
    if (sit == src_types.end() || *sit != _type)
      return kInvalidArchetypeId;

    std::vector<ComponentTypeId> dst;
    dst.reserve(src_types.size() - 1);
    for (auto t : src_types)
      if (t != _type) dst.push_back(t);

    ArchetypeId dst_id = this->GetOrCreate(dst);
    this->edges_[_src].remove.emplace(_type, dst_id);
    this->edges_[dst_id].add.emplace(_type, _src);
    return dst_id;
  }
}
