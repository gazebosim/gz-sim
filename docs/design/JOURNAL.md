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
