# Adjustable-Fidelity Initiative — Implementation Journal

Sequential record of what was built, which branches it landed on, what
tests were run, and any open questions for Nathan. Each phase gets a
dedicated section.

Branch naming: **`nkoenig/phase-<id>-<slug>`** in every repo a phase
touches. Gz-sim alone for 0a–0e and 1; multi-repo from phase 2 onward
(each repo gets the same branch name).

---

## Phase 0a — Archetype ECS core

**Branch:** `nkoenig/phase-0a-archetype-ecs` (in gz-sim)
**Design:** [phase-0a-archetype-ecm.md](phase-0a-archetype-ecm.md)
**Status:** Complete — all 8 milestones landed, 43 unit tests pass.

### Changes
- New standalone library `gz-sim-ecs` (static) under
  [gz-sim/src/ecs/](../../src/ecs/), public headers under
  [gz-sim/include/gz/sim/ecs/](../../include/gz/sim/ecs/). 5 public + 6
  internal detail headers, 8 `.cc` files.
- `ComponentTypeId` aliased to the existing `gz::sim::ComponentTypeId`;
  `TypeIdOf<T>()` SFINAE-prefers `T::typeId` (from
  `GZ_SIM_REGISTER_COMPONENT`) and falls back to `gz::common::hash64`.
- `EachParallel` uses `gz::common::WorkerPool` (one owned per World).
- 10 test binaries (43 cases) including a 5000-op fuzz test vs.
  `unordered_map` oracle.
- 4 benchmarks: `bench_iteration`, `bench_transitions`, `bench_parallel`,
  `bench_vs_ecm` (head-to-head vs classic `EntityComponentManager`).

### Tests
```
ctest -R UNIT_ECS_ --output-on-failure    # 43/43 passing
./bin/bench_vs_ecm 100000 20              # 49.87× speedup
./bin/bench_vs_ecm 1000000 10             # 8.21× speedup (above §12 gate)
./bin/bench_parallel                      # 0.86× linear at 8 workers
```

### Open questions (all resolved)
1. Type id stability → use existing `gz::sim::ComponentTypeId` (done).
2. `ModifyGuard<T>` proxy → out of 0a (done).
3. Parallel scaling gate → deferred to 0c (done).
4. `gz::common::WorkerPool` → adopted now (done).

### Commits
One squash commit at the end of this phase — see branch head.

---

## Phase 0b — ECM facade integration

**Branch:** `nkoenig/phase-0b-ecm-integration` (in gz-sim)
**Design:** [phase-0b-ecm-integration.md](phase-0b-ecm-integration.md)
**Status:** Foundation landed; full per-method facade migration deferred.

### Changes
- CMake option `GZ_SIM_ARCHETYPE_ECM` (default OFF) in
  [gz-sim/CMakeLists.txt](../../CMakeLists.txt:27); sets the
  `GZ_SIM_USE_ARCHETYPE_ECM=1` preprocessor define when ON.
- New facade class
  [`gz::sim::ecs::ArchetypeBackedECM`](../../include/gz/sim/ecs/ArchetypeBackedECM.hh)
  that exposes the ECM-shaped methods that in-tree systems actually use
  (CreateEntity, HasEntity, RequestRemoveEntity/Process, CreateComponent,
  RemoveComponent, Component, EntityHasComponent, Each / EachNew /
  EachChanged / EachRemoved, BeginPhase / CommitPhase,
  ClearNewlyCreatedEntities, ClearRemovedComponents,
  SetAllComponentsUnchanged). Backed by `gz::sim::ecs::World`. Handle
  translation table maintained internally so callers keep using
  `gz::sim::Entity`.
- 10 new unit tests in
  [test/ecs/ArchetypeBackedECM_TEST.cc](../../test/ecs/ArchetypeBackedECM_TEST.cc)
  covering the contract and confirming ECM-compatible semantics
  (including Each-callback-returns-false early-exit, deferred
  RequestRemoveEntity, non-trivially-relocatable component preservation
  across archetype transitions).

### What's explicitly deferred (not done this phase)
The full design calls for every one of ~40 `EntityComponentManager`
public methods to have a mirror in the archetype-backed `Impl` typedef,
plus conditional compilation of the ECM PIMPL to switch between
`LegacyImpl` and `ArchetypeImpl`. That is a multi-week engineering job
(per the design's Week 1–8 plan). What shipped this session:
- The architectural toggle, build integration, and target facade class.
- The contract that `CommitPhase()` is the end-of-phase boundary marker
  (calls `World::Commit()`; no-op if `World` isn't in-phase).
- Parity proof-of-concept via the scripted-sequence test.

What's **not** shipped this session and needs a dedicated follow-up
session to finish 0b per its §15 milestone table:
- State / SetState / ChangedState serialization round-trip.
- View API (`BaseView` uses) — legacy-only, archetype core uses queries.
- Clone / ResetTo / CopyFrom.
- EntityGraph / Parent-child helpers (`ChildrenByComponents`, etc.) —
  `components::ParentEntity` still works but the graph accelerators
  aren't wired.
- SimulationRunner hooks that actually call `CommitPhase()` between
  PreUpdate/Update/PostUpdate — the API exists but no callsite yet
  invokes it under legacy builds.
- `EntityComponentManager_TEST` under `GZ_SIM_ARCHETYPE_ECM=ON` — the
  existing 50+ tests use surface we haven't mirrored yet.

### Tests
```
./bin/UNIT_ECS_ArchetypeBackedECM_TEST   # 10 tests, all green
```

### Open questions for Nathan
**Q1.** Should I push forward and ship the full-facade migration (the
remaining ~30 methods of the ECM) in a follow-up session, or is the
Phase 0a+foundation proof enough for you to have gained confidence and
we stage the big migration as a separate, properly-reviewed engineering
effort? (Recommended: stage separately — the risk isn't in the facade
design but in the migration execution across the ~50 test surfaces the
legacy ECM touches.)

**Q2.** The `ArchetypeBackedECM` maintains a legacy↔core handle
translation table. In the design this is described as transitional. Do
you want me to remove it by refitting `gz::sim::Entity` to carry the
generation bits directly (ABI break for any downstream that reads the
entity integer), or leave the translation in place and drop it in 0e?

---
