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

## Phase 0b — Real facade scaffolding (2026-04-20 update)

**Branch:** `main` (merged forward of `nkoenig/phase-0b-ecm-integration`)
**Scope:** V2 architecture from design §3.1, 1b mutation semantics
from §6.1, option (a) for not-yet-ported methods.

### What changed in the code
- **`include/gz/sim/EntityComponentManager.hh`** — minimal additions:
  forward-decl of `ecs::World` and `detail_archetype::FromCore`;
  include of the tiny `gz/sim/ecs/Entity.hh`; two new private
  accessor declarations `ArchetypeWorld()` (const + non-const); a
  friend declaration; and the single `#if defined(GZ_SIM_USE_ARCHETYPE_ECM)`
  at the end of the file that switches between the legacy detail
  template header and the new archetype one. No changes to any
  template method body or public signature.
- **`include/gz/sim/detail/EntityComponentManagerArchetype.hh`** —
  new, parallel to the existing
  `include/gz/sim/detail/EntityComponentManager.hh` (which stays
  untouched). Contains archetype-backend template bodies for
  `CreateComponent<T>`, `Component<T>` (const/mutable),
  `ComponentDefault<T>`, `ComponentData<T>`, `SetComponentData<T>`,
  `RemoveComponent<T>`, `Each<T...>` (both overloads),
  `EachNew<T...>` (both), `EachRemoved<T...>`, `EachNoCache<T...>`
  (both), `EntityByComponents<T...>`, `EntitiesByComponents<T...>`,
  `ChildrenByComponents<T...>`, `ForEach<Fn, Ts...>`. These call
  `ecs::World::Each<T...>` directly — no `detail::View` machinery.
- **`src/EntityComponentManagerArchetype.cc`** — new, parallel to the
  existing `src/EntityComponentManager.cc` (which stays untouched per
  explicit direction). Defines `EntityComponentManagerPrivate`
  (wrapping `ecs::World` + the legacy↔core entity translation table +
  the existing `EntityGraph` for parent/child), provides real
  implementations for ~20 methods (ctor, CreateEntity, HasEntity,
  ComponentImplementation, CreateComponentImplementation [STUB],
  RemoveComponent(typeId), Entities, ParentEntity, SetParentEntity,
  Descendants, change-tracking clears, Request/Process remove,
  Pin/Unpin, the archetype accessors), and stubs (gzerr-once +
  sensible default) for the remaining ~25 — each tagged
  `// STUB(0b):` and findable via `grep -rn "STUB(0b)"`.
- **`src/EntityComponentManagerStubs.cc`** — new, legacy-build-only.
  Provides the `ArchetypeWorld()` + `detail_archetype::FromCore`
  symbols so the legacy `libgz-sim.so` links without touching the
  existing legacy `.cc`. Returns nullptr / kNullEntity; the paths
  are never hit under the legacy backend.
- **`include/gz/sim/ecs/World.hh`** — promoted `ComponentRaw` /
  `ComponentRawMut` from private to public and added `RemoveRaw` so
  the non-template ECM facade can reach component data and remove
  by typeId.
- **`src/ecs/World.cc`** — new `RemoveRaw(e, typeId)` that routes
  through `Immediate/DeferredRemove` based on phase state.
- **`src/CMakeLists.txt`** — source-file split: flag OFF →
  `EntityComponentManager.cc` + `EntityComponentManagerStubs.cc`
  (preserves legacy byte-for-byte); flag ON →
  `EntityComponentManagerArchetype.cc`. Separately, links the
  `gz-sim-ecs` static library into `libgz-sim` only when the flag
  is ON.

### What's explicitly honored
- **Legacy `.cc` untouched.** `git diff HEAD~1 src/EntityComponentManager.cc`
  returns empty on the legacy path.
- **V2 approach.** Exactly one `#if` in the code path, at the single
  detail-header include site. Both `.cc` files read as linear,
  non-preprocessor-branching implementations.
- **Option 1 for templates.** Simple templates forward through
  type-erased `*Implementation()` helpers; view-driven templates call
  `ecs::World::Each<T...>` directly. No `detail::View` duplication.
- **1b semantics.** `CreateComponent` / `RemoveComponent` apply
  immediately on the main thread; mutations inside `EachParallel`
  bodies defer to the thread-local command buffer (archetype core
  does this already). Every immediate-path call carries
  `// NOTE(0b-immediate-mode):` — Phase 0c migrates to
  strictly-deferred and documented that in
  [phase-0c-system-port.md](phase-0c-system-port.md) §4.0.
- **Option (a) for unimplemented.** Every stub logs once via
  `gzwarn` with a pointer to the design doc, then returns the
  sensible default.

### Build + test verification
Both modes build clean:
```
cmake -DGZ_SIM_ARCHETYPE_ECM=OFF ..  &&  cmake --build . --target gz-sim  # legacy
cmake -DGZ_SIM_ARCHETYPE_ECM=ON  ..  &&  cmake --build . --target gz-sim  # archetype
```

Legacy build test surface — **no regressions**:
```
UNIT_EntityComponentManager_TEST    414/414 green
UNIT_ECS_* (12 binaries)            55/55 green
```

Archetype build test surface — **library links and loads**, but the
full `UNIT_EntityComponentManager_TEST` hits the `STUB(0b)` paths
(State / SetState / Clone / …) so most of its 400+ cases fail at the
archetype build. That's expected and documented — Phase 0b completion
is the port that fills in those stubs.

### Known gaps / Phase 0b completion work
Grep-able list for the eventual port session:
```
grep -rn "STUB(0b)" src/ include/
grep -rn "NOTE(0b-immediate-mode)" src/ include/
```
- `CreateComponentImplementation` (non-template path) currently logs
  and no-ops. Template `CreateComponent<T>(e, T{data})` **works** —
  it goes through `World::Add<T>`. Non-template callers (which are
  rare but exist in serialization) hit the stub. Fix: add a
  type-erased `World::AddRaw(entity, typeId, const void*)` that uses
  the ComponentTypeRegistry's copy-construct thunk.
- Serialization methods (`State`, `SetState`, `ChangedState`,
  `PeriodicStateFromCache`, `AddEntityToMessage`) all stub. The
  archetype core's change-bits + removed-records have the needed
  info; the port is mostly transcription.
- `Clone` / `CopyFrom` / `ResetTo`. Needs the archetype-world
  column-wise copy loop described in design §4.4.
- `EntityByName` needs a name-index which the archetype PIMPL
  doesn't currently maintain — add an unordered_map keyed on the
  `components::Name` component value.
- `HasNewEntities`, `HasOneTimeComponentChanges`,
  `HasPeriodicComponentChanges` need aggregate flags on `ecs::World`.
- `IsNewEntity` needs a "row was added this step" lookup. The
  archetype core's per-archetype `newly_added` vector has the info;
  needs a typed accessor.

### Verification that `gz sim` is unchanged
The installed `libgz-sim.so` is the legacy build (flag OFF). `gz sim
shapes.sdf` uses identical code paths to before this change — only
delta is that two new symbols are exported (the stub `ArchetypeWorld`
accessors). Measured `nm -D` confirms. `gz sim` behavior: unchanged.

### Open questions for Nathan (carried forward — new ones)
**Q20.** The archetype facade doesn't yet maintain its own name-index,
so `EntityByName` stubs to `nullopt`. Do you want that landed in this
0b-completion pass, or can `SdfEntityCreator` opt into providing it
explicitly (it already keeps a name map internally)?

**Q21.** Several callers of ECM in-tree serialization use
`AddEntityToMessage` directly (private-but-friend from
`SceneBroadcaster` or similar). Those paths will break under the
archetype backend until the serialization stubs are filled in. Do we
(a) make `SceneBroadcaster` opt-out under archetype (scene mirror only
emits the state delta it computes itself), (b) get the serialization
stubs green before flipping any test matrix, or (c) ignore until a
real user hits it?

---
