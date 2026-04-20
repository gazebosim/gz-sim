/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include "gz/sim/ecs/ArchetypeBackedECM.hh"

#include <algorithm>

namespace gz::sim::ecs
{
  ArchetypeBackedECM::ArchetypeBackedECM() = default;
  ArchetypeBackedECM::~ArchetypeBackedECM() = default;

  gz::sim::Entity ArchetypeBackedECM::CreateEntity()
  {
    gz::sim::ecs::Entity h = this->world_.CreateEmpty();
    gz::sim::Entity legacy = this->next_legacy_++;
    this->by_legacy_.emplace(static_cast<uint64_t>(legacy), h);
    this->by_core_.emplace(h.Raw(), legacy);
    return legacy;
  }

  bool ArchetypeBackedECM::HasEntity(gz::sim::Entity _e) const
  {
    auto it = this->by_legacy_.find(static_cast<uint64_t>(_e));
    if (it == this->by_legacy_.end()) return false;
    return this->world_.IsAlive(it->second);
  }

  void ArchetypeBackedECM::RequestRemoveEntity(gz::sim::Entity _e)
  {
    this->pending_removals_.push_back(_e);
  }

  void ArchetypeBackedECM::ProcessRemoveEntityRequests()
  {
    for (auto e : this->pending_removals_)
    {
      auto it = this->by_legacy_.find(static_cast<uint64_t>(e));
      if (it == this->by_legacy_.end()) continue;
      auto core = it->second;
      this->world_.Destroy(core);
      this->by_core_.erase(core.Raw());
      this->by_legacy_.erase(it);
    }
    this->pending_removals_.clear();
  }

  void ArchetypeBackedECM::BeginPhase()
  {
    this->world_.BeginPhase();
  }

  void ArchetypeBackedECM::CommitPhase()
  {
    this->world_.Commit();
  }

  void ArchetypeBackedECM::SetAllComponentsUnchanged()
  {
    this->world_.ClearChangeBits();
  }

  void ArchetypeBackedECM::ClearNewlyCreatedEntities()
  {
    // The archetype core's ClearChangeBits clears both newly-added and
    // dirty bits. Legacy semantics separate them; since callers always
    // run both clears at the same boundary (end of step), this is
    // observationally equivalent in tested scenarios. Left as a 0c
    // split-out if a use case needs finer granularity.
    this->world_.ClearChangeBits();
  }

  void ArchetypeBackedECM::ClearRemovedComponents()
  {
    // Removal records live on the World — clearing change bits drains
    // them. Kept as a distinct method for API parity.
    this->world_.ClearChangeBits();
  }

  size_t ArchetypeBackedECM::EntityCount() const
  {
    return this->world_.NumEntities();
  }

  gz::sim::ecs::Entity ArchetypeBackedECM::Handle(gz::sim::Entity _e) const
  {
    auto it = this->by_legacy_.find(static_cast<uint64_t>(_e));
    if (it == this->by_legacy_.end())
      return gz::sim::ecs::kNullEntity;
    return it->second;
  }

  gz::sim::Entity ArchetypeBackedECM::LegacyHandle(
      gz::sim::ecs::Entity _h) const
  {
    auto it = this->by_core_.find(_h.Raw());
    if (it == this->by_core_.end()) return gz::sim::kNullEntity;
    return it->second;
  }
}
