/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * Legacy-build-only stubs for the ECM archetype accessors declared in
 * include/gz/sim/EntityComponentManager.hh.
 *
 * The archetype backend (src/EntityComponentManagerArchetype.cc) has
 * its own real definitions of `ArchetypeWorld()` and
 * `detail_archetype::FromCore()`. Under the legacy build
 * (`GZ_SIM_ARCHETYPE_ECM=OFF`) we still need symbol definitions so the
 * public header's private accessor links — this file provides them,
 * each returning the "no archetype here" sentinel. Per design, this
 * avoids any `#ifdef` inside the existing legacy implementation file.
 *
 * See docs/design/phase-0b-ecm-integration.md §3.1 for architecture.
 */
#include "gz/sim/EntityComponentManager.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  ecs::World *EntityComponentManager::ArchetypeWorld()
  {
    // Legacy build: no archetype backend exists. Callers (the archetype
    // detail header templates) check for null and no-op. Under the
    // legacy build the templates in that header aren't even instantiated
    // — the legacy detail header is used instead — so this path is
    // defensive only.
    return nullptr;
  }

  const ecs::World *EntityComponentManager::ArchetypeWorld() const
  {
    return nullptr;
  }

  namespace detail_archetype
  {
    gz::sim::Entity FromCore(
        const EntityComponentManager * /*_ecm*/,
        gz::sim::ecs::Entity /*_core*/)
    {
      // Same defensive reasoning: under the legacy build this is
      // never called. Return sentinel.
      return kNullEntity;
    }
  }
}
}
}
