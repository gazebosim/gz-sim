/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#include "gz/sim/ecs/detail/ArchetypeGraph.hh"

#include <algorithm>
#include <cassert>
#include <stdexcept>

namespace gz::sim::ecs
{
  ArchetypeGraph::ArchetypeGraph()
  {
    // Pre-create the empty archetype (ArchetypeId 0). Used for entities with
    // no components yet.
    this->AllocateArchetype({});
  }

  ArchetypeGraph::~ArchetypeGraph() = default;

  Archetype &ArchetypeGraph::Get(ArchetypeId _id)
  {
    return *this->archetypes_[_id];
  }

  const Archetype &ArchetypeGraph::Get(ArchetypeId _id) const
  {
    return *this->archetypes_[_id];
  }

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

  ArchetypeId ArchetypeGraph::AddEdge(ArchetypeId _src, ComponentTypeId _type)
  {
    {
      auto it = this->edges_[_src].add.find(_type);
      if (it != this->edges_[_src].add.end())
        return it->second;
    }

    // Work with a local copy of src_types — GetOrCreate below may grow
    // archetypes_/edges_, invalidating references.
    std::vector<ComponentTypeId> src_types = this->archetypes_[_src]->Types();
    std::vector<ComponentTypeId> dst;
    dst.reserve(src_types.size() + 1);
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
    // Re-fetch edges_ slots after potential growth.
    this->edges_[_src].add.emplace(_type, dst_id);
    this->edges_[dst_id].remove.emplace(_type, _src);
    return dst_id;
  }

  ArchetypeId ArchetypeGraph::RemoveEdge(
      ArchetypeId _src, ComponentTypeId _type)
  {
    {
      auto it = this->edges_[_src].remove.find(_type);
      if (it != this->edges_[_src].remove.end())
        return it->second;
    }

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
