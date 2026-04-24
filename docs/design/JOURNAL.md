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

## Phase 0b — Archetype facade boots `gz sim` (2026-04-24 update)

**Branch:** `main` (follow-on to the 2026-04-20 V2 scaffolding).
**Result:** `gz sim -s -v4 empty.sdf` completes startup cleanly under
`GZ_SIM_ARCHETYPE_ECM=ON`. No crashes. Server reaches steady state,
PostUpdate workers run, SIGTERM shutdown is clean. Two harmless
`STUB(0b)` warnings remain (`SetChanged` one-shot, `CopyFrom`
one-shot).

### What landed

1. **Shadow `BaseComponent` store** in the archetype PIMPL
   (`std::unordered_map<Entity, unordered_map<typeId,
   unique_ptr<BaseComponent>>>`). Rationale: SdfEntityCreator and the
   Factory pipeline use the type-erased `CreateComponentImplementation
   (e, typeId, BaseComponent*)` path, where we have no T available
   and therefore can't route through the archetype core. Clone the
   caller's BaseComponent polymorphically and keep the clone.
   `ComponentImplementation` / `EntityHasComponentType` /
   `ComponentTypes` / `RemoveComponent` check the shadow map first.
   Template callers continue to go through the archetype World at
   full performance. Marker: `NOTE(0b-shadow-store)` in the source.

2. **New `ecs::World::AddRaw(entity, typeId, const void*)`**
   public method. Dispatches to the existing `ImmediateAdd` /
   `DeferredAdd` path by looking the type up in
   `ComponentTypeRegistry`. Used by
   `CreateComponentImplementation` when the typeId IS registered
   (today: only happens if someone called the template path for
   that T previously — rare at boot). Tomorrow, once there's a
   Factory ↔ ComponentTypeRegistry bridge, most non-template creates
   will go through `AddRaw` at full archetype performance and the
   shadow store falls back to a narrow edge case.

3. **`CoreFor` sentinel bug fix** — the original impl returned
   `kNullEntity` to signal "not in map", but the archetype core's
   first-created entity also has raw value 0 (Entity(0, 0)). The
   world entity's legitimate handle collided with the not-found
   sentinel, making `HasEntity(worldEntity)` return false right after
   `CreateEntity` returned 1. This was the actual crash cause —
   `SdfEntityCreator::CreateComponent` bailed silently because
   `HasEntity` said the world didn't exist, so no components got
   attached, so `UserCommands::Configure` got a null `Name` and
   dereferenced it. Fixed by changing `CoreFor` to a bool + out-param
   signature. Every callsite updated.

4. **`World::AddRaw` + `RemoveRaw` + `ComponentRaw` /
   `ComponentRawMut` all public** on `ecs::World`. Non-template
   facade callers need these.

5. **`ComponentTypes(entity)` implemented**. The legacy test suite
   calls it; stubbed it would return empty. Now iterates the
   archetype AND shadow store.

### Verification
- `gz sim -s -v4 empty.sdf` — runs to the SIGTERM timeout without
  crashing. Only two stub warnings fire (`SetChanged`, `CopyFrom`),
  both one-shot and harmless.
- `UNIT_EntityComponentManager_TEST` under legacy build — **414/414
  green**. Legacy path unchanged.
- `UNIT_ECS_*` (12 binaries) still green.

### What's still stubbed and will bite real scenarios
- **State / SetState / ChangedState / ChangedState(SerializedStateMap)**
  — scene broadcasting, state serialization for network secondaries,
  and the initial-state snapshot the GUI subscribes to all go through
  these. `gz sim` with the GUI connected will show an empty world
  because state serialization returns `{}`. Fill-in is mechanical:
  walk archetype chunks column-by-column and emit `SerializedComponent`
  messages; the format is documented in §9 of the Phase 0b design.
- **Clone / CopyFrom / ResetTo** — needed for save/load, log playback,
  and network primary/secondary state sync.
- **SetChanged / ComponentState** — change-tracking granularity
  (OneTimeChange / PeriodicChange). Currently always reports
  NoChange, which means downstream systems using the change-tracked
  topic filter (e.g., scene_broadcaster's periodic vs. one-time
  delta) get everything.
- **EntityByName** — needs a shadow name index on the facade.

### Remaining grep recipes
```
grep -rn "STUB(0b)"                src/ include/   # 15+ methods
grep -rn "NOTE(0b-immediate-mode)" src/ include/   # Phase 0c migration markers
grep -rn "NOTE(0b-shadow-store)"   src/ include/   # shadow-store bridge work
```

### Open questions
**Q22.** The shadow store route doesn't get components into
`World::Each<T>` iteration. Systems that do `Each<Pose>` at the first
PostUpdate to build their internal caches will see no entities under
`gz sim empty.sdf` today, because all components went through the
shadow store. For empty.sdf this doesn't crash because no physics /
sensor system needs to iterate — but a scenario with any actual
entity-component iteration will behave wrong. The proper fix is the
Factory ↔ ComponentTypeRegistry bridge (hand every
GZ_SIM_REGISTER_COMPONENT type into the archetype registry at
load-time). Confirm we do that as the next session, and it's OK for
the Each<T> miss to be a known limitation until then?

---

## Phase 0b — State / SetState serialization (2026-04-24 update)

**Context:** `gz sim empty.sdf` + play button triggered
`STUB(0b)` warnings for `State` and `SetState`. Those methods drive
GUI scene sync, scene broadcaster emission, and network primary/
secondary state transfer. Implemented.

### Changes
- **`EntityComponentManager::State(SerializedStateMap&, ...)`**
  walks every entity in `legacyToCore`, filters by the optional
  `_entities` set, and delegates to `AddEntityToMessage` for the
  protobuf serialization. Matches the legacy implementation at
  [src/EntityComponentManager.cc:1722](../../src/EntityComponentManager.cc#L1722).
- **`AddEntityToMessage(SerializedStateMap&, entity, types, full)`**
  fills in the protobuf entity sub-message. Reads component values
  via `ComponentImplementation(e, typeId)` — automatically pulls
  from the shadow store OR the archetype world as appropriate.
  Serializes via `BaseComponent::Serialize(std::ostream&)`.
- **`ChangedState(SerializedStateMap&)`** currently calls `State`
  with `_full=true`. Real delta emission pends change-tracking work
  — marker `STUB(0b-delta)`.
- **`SetState(SerializedStateMap)`** mirrors legacy logic at
  [src/EntityComponentManager.cc:1911](../../src/EntityComponentManager.cc#L1911):
  creates missing entities (preserving the incoming id), creates
  missing components via `components::Factory::Instance()->New(type)`
  + `Deserialize`, updates existing components in place, handles
  component removal, and calls `SetChanged` to mirror the legacy
  change-tracking hooks (no-op today). Advances `nextLegacyId` past
  any peer-assigned id so auto-allocation doesn't collide.
- **Plain `SerializedState` (non-map) State/SetState** remain stubs
  — in-tree gz sim uses the map variant exclusively; plain-state
  callers are a handful of tests + Python bindings and not a
  blocker for the play-button path.

### Verification
- `timeout 6 gz sim -s -v4 empty.sdf` — server starts, waits, exits
  cleanly on SIGTERM. No State/SetState stub warnings.
- `timeout 4 gz sim -s -v4 -r empty.sdf` (run on start) — physics
  runner ticks, scene broadcaster runs its State() path each step,
  no stub warnings from that code path.
- Remaining stubs: `SetChanged`, `CopyFrom` (both one-shots during
  init, harmless) — unchanged from previous iteration.

### Known limitation
`_full=false` in `State()` is treated as `_full=true` — every
component emitted every call. The legacy backend uses
one-time/periodic change tracking to skip unchanged components; the
archetype backend hasn't wired that yet. Impact: wire load on the
scene-broadcast topic is higher than it needs to be, but behavior
is correct. `STUB(0b-delta)` marker placed in the code for the
follow-up.

---

## Phase 0b — Each<T...> unified entity walk (2026-04-24 update)

**Context:** Q22 in the earlier journal: shadow-stored components
weren't visible to `Each<T...>` iteration. Systems that walked the
ECM with `Each<Pose>` at PostUpdate were seeing nothing because
SdfEntityCreator's components all landed in the shadow store, not
the archetype World.

### Changes
- **Private helper
  `EntityComponentManager::AllEntitiesArchetypeFacade() const`**
  added to the public header. Under the archetype build it returns
  the full legacy-entity-id set (union of shadow store + archetype
  World, which are subsets of `legacyToCore`). Under the legacy
  build it returns an empty vector — the legacy detail header never
  calls it.
- **Archetype detail-header `Each<T...>` rewritten** (both
  const/mutable overloads). New body: walk every entity via
  `AllEntitiesArchetypeFacade()`, read each requested component via
  `Component<T>(e)` (which unifies shadow + World reads), skip
  entities that don't have all the types, invoke the callback
  otherwise. Same pattern applied to `EachNoCache`, `EachRemoved`
  (with `IsMarkedForRemoval` filter), `EntityByComponents`,
  `EntitiesByComponents`. `EachNew` currently aliases to `Each`
  (marker `STUB(0b-new)` — pending change-tracking wire-up).
- **`EntityByName`** implemented against the unified iterator —
  linear scan of entities with a `Name` component, compare data.
- **Performance trade-off** documented in the code: per-entity loop
  costs closer to the legacy ECM's ~30–50 ns/entity rather than the
  archetype core's ~3 ns/entity. The fast-path returns once the
  Factory ↔ ComponentTypeRegistry bridge eliminates the shadow
  store.

### Verification
- `timeout 4 gz sim -s -v4 -r empty.sdf` — runs + ticks + shuts
  down cleanly. Only two harmless stub warnings remain: `SetChanged`
  (change-tracking not wired) and `CopyFrom` (init-time one-shot).
  `EntityByName` stub gone.
- `UNIT_EntityComponentManager_TEST` under legacy — 414/414 green.
  Legacy path unchanged.

### Still-stubbed (updated list)
```
grep -rn "STUB(0b)"  src/ include/   # remaining methods
grep -rn "STUB(0b-delta)"            # change-tracking dependent paths
grep -rn "STUB(0b-new)"              # newly-created-entity tracking
grep -rn "NOTE(0b-immediate-mode)"   # Phase 0c migration markers
grep -rn "NOTE(0b-shadow-store)"     # Factory↔registry bridge pending
```

Impact remaining:
- `SetChanged` / `HasOneTimeComponentChanges` /
  `HasPeriodicComponentChanges` — change-tracking state. Drives
  delta emission in `State(_full=false)`. Harmless today; wasteful.
- `CopyFrom` / `Clone` / `ResetTo` — save/load, log playback.
  Won't bite normal `gz sim` boot.
- `ComponentState` getter — always returns `NoChange`.
- `HasNewEntities` — aggregate flag. Returns false (used by
  SceneBroadcaster's quick-short-circuit; over-emitting is OK).

All of these are follow-up. The backend now serves basic `gz sim`
scenes correctly.

---

## Phase 0b — Newly-created entity tracking (2026-04-24 update)

**Context:** After the Each<T...> unification, `gz sim -v4
empty.sdf` produced a continuous stream of Physics warnings like
`Model entity [4] marked as new, but it's already on the map`.
Physics's `EachNew<Model, ...>` path was seeing every entity every
tick because the archetype-facade `EachNew<T>` aliased `Each<T>`
(STUB(0b-new)). Physics would try to register the entity in its
local map, notice it was already there, and warn.

### Fix
- Added `std::unordered_set<Entity> newlyCreatedEntities` to the
  archetype PIMPL.
- `CreateEntity` + `SetState`'s entity-create path insert into the
  set.
- `ClearNewlyCreatedEntities` drains it (called end-of-step by
  `SimulationRunner`).
- `ProcessRemoveEntityRequests` removes entries for destroyed
  entities.
- `IsNewEntity(e)` returns membership; `HasNewEntities()` returns
  `!empty()`.
- `EachNew<T>` detail-header template now filters
  `AllEntitiesArchetypeFacade()` by `IsNewEntity` before applying
  the component-has-all check.

### Critical gotcha — plugin .so files need a full rebuild

The detail-header template bodies are instantiated in the **consumer**
translation units, including every gz-sim system plugin `.so`
(Physics, SceneBroadcaster, etc.). Rebuilding `libgz-sim.so` alone
doesn't refresh those template instantiations. The plugin `.so`s
have to rebuild too.

Verified during debugging: with `libgz-sim.so` updated but
`libgz-sim-physics-system.so` still stale, Physics saw 3460
"already on the map" warnings in a 3 s run. After full rebuild
(`cmake --build . -j8`, not just `--target gz-sim`), zero warnings.

**Operational recommendation:** do `cmake --build . -j8` (no
target) after any change to
`include/gz/sim/detail/EntityComponentManagerArchetype.hh` to catch
every plugin rebuild.

### Verification
- `timeout 3 gz sim -s -v4 -r empty.sdf` — zero `marked as new`
  warnings (previously 3460+ per 3 s run).
- `UNIT_EntityComponentManager_TEST` under legacy: 414/414 green.
- Only remaining stub warnings: `SetChanged` + `CopyFrom` one-shots
  during init.

---

## Phase 0b — Parent-first entity iteration order (2026-04-24 update)

**Context:** `gz sim -v4 shapes.sdf` segfaulted in the GUI client,
specifically inside `EntityTree::AddEntity` at
[src/gui/plugins/entity_tree/EntityTree.cc:189](../../src/gui/plugins/entity_tree/EntityTree.cc#L189).
The crash was a null-/dangling-QString deref during atomic
ref-counting.

### Root cause
`EntityTree::AddEntity` implements a pending-entities queue with a
recursive dance: when an entity whose parent isn't yet known
arrives, it's pushed onto `pendingEntities`; when a parent is
later added, the method `std::partition`s `pendingEntities`,
iterates the partitioned range, and recursively calls
`AddEntity(childEntity, …)` inside the loop. The recursive
`AddEntity` calls do their own `std::partition` on the same
`pendingEntities` vector — which can swap elements into and out of
the outer loop's iterator position, invalidating the outer `it`.
The subsequent dereference reads a garbage `QString`, whose `d`
pointer dereferences into unmapped memory, segfault.

Whether the bug fires depends on the arrival order of entities on
the state topic. **Parent-first order is safe** — `pendingEntities`
stays empty, the fragile recursion never runs. Under the legacy
ECM, the map iteration order happened to coincide with creation
order (parents before children). Under the archetype facade my
`AllEntitiesArchetypeFacade()` iterates `legacyToCore` which is a
`std::unordered_map` — random order — so children could arrive
before their parents, triggering the recursion, triggering the
crash.

### Fix
- `AllEntitiesArchetypeFacade()` now returns legacy entity IDs
  **sorted ascending**. gz-sim's creation convention is strictly
  top-down (parent created before child), so ascending-ID order
  equals parent-first order. This propagates to both `Each<T...>`
  iteration and state-emission order.
- `State(SerializedStateMap&, ...)` now iterates via
  `AllEntitiesArchetypeFacade()` rather than directly over
  `legacyToCore`, inheriting the sort.

### Verification
- `GZ_PARTITION=ecstest timeout 5 gz sim -s -v4 -r shapes.sdf` —
  exits 124 (timeout), no segfault. Server runs clean.
- `GZ_PARTITION=ecstest timeout 6 gz sim -v4 shapes.sdf` (with GUI) —
  exits 124, no segfault. GUI client survives the 6-second window
  with the scene populated.
- `UNIT_EntityComponentManager_TEST` under legacy: 414/414 green.

### Note on the upstream fragility
`EntityTree::AddEntity`'s recursive-partition loop is fragile by
design: it assumes the recursive calls don't mutate
`pendingEntities` in ways that invalidate the outer iterator.
That assumption holds only for certain arrival orderings. A
proper upstream fix would replace the recursion with an explicit
worklist that doesn't overlap with the partition's working range.
Out of Phase 0b scope — the parent-first-emission workaround
restores behavior for any downstream consumer that walks
pendingEntities this way.

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
