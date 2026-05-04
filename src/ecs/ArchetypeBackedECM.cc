/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */

// ArchetypeBackedECM — standalone scaffold class wrapping `ecs::World`
// behind an ECM-shaped façade.
//
// This is the proof-of-concept class shipped alongside the archetype
// core during Phase 0b's bring-up — it exposes a small subset of the
// `EntityComponentManager` surface (CreateEntity, HasEntity,
// RequestRemoveEntity / Process, BeginPhase / CommitPhase, the
// change-tracking sweepers) backed by an `ecs::World`. It is used by
// the parity tests in `test/ecs/` to verify that the archetype core
// can model ECM semantics without touching the production ECM code.
//
// **This is NOT the production ECM facade.** The real facade lives
// in `src/EntityComponentManagerArchetype.cc` and replaces
// `EntityComponentManager` wholesale when built with
// `GZ_SIM_ARCHETYPE_ECM=ON`. Keep this class as a compact reference
// implementation; do not extend it to mirror every ECM method.
//
// Legacy↔core handle translation: `ecs::Entity` is a 64-bit
// generation-tagged handle; `gz::sim::Entity` is a plain monotonic
// uint64. We maintain two parallel maps so callers continue to use
// `gz::sim::Entity` while internal storage uses `ecs::Entity`.
#include "gz/sim/ecs/ArchetypeBackedECM.hh"

#include <algorithm>

namespace gz::sim::ecs
{
  ArchetypeBackedECM::ArchetypeBackedECM() = default;
  ArchetypeBackedECM::~ArchetypeBackedECM() = default;

  // Allocate a core entity, mint a fresh monotonic legacy id, and
  // register both directions of the translation map. The legacy id
  // is what callers see; the core handle is what we hand to `world_`
  // for storage operations.
  gz::sim::Entity ArchetypeBackedECM::CreateEntity()
  {
    gz::sim::ecs::Entity h = this->world_.CreateEmpty();
    gz::sim::Entity legacy = this->next_legacy_++;
    this->by_legacy_.emplace(static_cast<uint64_t>(legacy), h);
    this->by_core_.emplace(h.Raw(), legacy);
    return legacy;
  }

  // True if the legacy id resolves to a core handle that's still
  // alive in `world_`. Returns false for unknown ids and for
  // already-destroyed entities.
  bool ArchetypeBackedECM::HasEntity(gz::sim::Entity _e) const
  {
    auto it = this->by_legacy_.find(static_cast<uint64_t>(_e));
    if (it == this->by_legacy_.end()) return false;
    return this->world_.IsAlive(it->second);
  }

  // Queue an entity for deferred removal. Mirrors legacy ECM: the
  // request is held until `ProcessRemoveEntityRequests` runs (typically
  // at end-of-step from `SimulationRunner`).
  void ArchetypeBackedECM::RequestRemoveEntity(gz::sim::Entity _e)
  {
    this->pending_removals_.push_back(_e);
  }

  // Drain the pending-removal queue. For each id, look up the core
  // handle, destroy it on `world_`, and erase both translation
  // entries. Skips ids we don't recognize — the legacy ECM tolerates
  // requests for unknown entities.
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

  // Forward to the core. SimulationRunner brackets each system phase
  // with these calls.
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

  // The core's `ClearChangeBits` clears both newly-added and
  // dirty bits in one pass. Legacy ECM separates them, but in
  // every tested call site both clears run at the same step
  // boundary, so collapsing them here is observationally
  // equivalent. Split into distinct calls if a use case ever
  // needs finer granularity.
  void ArchetypeBackedECM::ClearNewlyCreatedEntities()
  {
    this->world_.ClearChangeBits();
  }

  // Removal records live on `world_`; the same `ClearChangeBits`
  // call drains them. Kept as a distinct method for API parity
  // with the legacy ECM.
  void ArchetypeBackedECM::ClearRemovedComponents()
  {
    this->world_.ClearChangeBits();
  }

  size_t ArchetypeBackedECM::EntityCount() const
  {
    return this->world_.NumEntities();
  }

  // Look up a core handle for a legacy id. Returns `kNullEntity` if
  // the id was never registered. Used by the templated component
  // accessors that route through `world_`.
  gz::sim::ecs::Entity ArchetypeBackedECM::Handle(gz::sim::Entity _e) const
  {
    auto it = this->by_legacy_.find(static_cast<uint64_t>(_e));
    if (it == this->by_legacy_.end())
      return gz::sim::ecs::kNullEntity;
    return it->second;
  }

  // Inverse of `Handle`: legacy id for a core handle. Used by the
  // `Each` templates when the core hands back an `ecs::Entity` and
  // the caller needs the legacy id to plug back into other ECM APIs.
  gz::sim::Entity ArchetypeBackedECM::LegacyHandle(
      gz::sim::ecs::Entity _h) const
  {
    auto it = this->by_core_.find(_h.Raw());
    if (it == this->by_core_.end()) return gz::sim::kNullEntity;
    return it->second;
  }
}
