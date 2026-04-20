# Phase 0b — Integrating the Archetype Core Behind the ECM Facade

Status: Draft
Owner: Nathan Koenig
Scope: Phase 0b of the Adjustable-Fidelity initiative
Depends on: [0a](phase-0a-archetype-ecm.md) (archetype core library)
Blocks: 0c (in-tree system port), 0d (YAML Configure), 0e (flip default)

## 1. Context

Phase 0a landed `gz::sim::ecs::World` as a standalone archetype ECS library
with a proven iteration gate. Phase 0b threads it under the existing
`gz::sim::EntityComponentManager` (ECM) public API so the rest of `gz-sim`
continues to compile and pass tests without per-system edits.

The existing ECM header is
[gz-sim/include/gz/sim/EntityComponentManager.hh](../../include/gz/sim/EntityComponentManager.hh)
(852 lines, ~40 public methods). 0b is about preserving the semantics of
every one of those methods on top of the archetype core — with the single
planned exception of pointer-validity (§6).

Deliverable: a build toggle `GZ_SIM_ARCHETYPE_ECM` (default OFF in 0b, ON in
0e) that selects the backend at compile time. Both backends coexist; both
pass the full test suite.

## 2. Goals / Non-goals

Goals:
- `EntityComponentManager` becomes a thin facade delegating to either the
  legacy storage or `ecs::World`, chosen at build time.
- All public methods preserve behavior modulo the documented pointer-validity
  contract shift.
- All existing gz-sim unit + integration tests pass under both backends.
- Serialized state (`msgs::SerializedState`, `msgs::SerializedStateMap`)
  round-trips identically.
- `SdfEntityCreator`, `LevelManager`, `SimulationRunner`, `SystemManager`,
  `Scene3d`, `SceneBroadcaster`, networking primary/secondary paths — all
  keep working untouched.

Non-goals:
- Porting individual systems to use the new `ecs::World` API directly — that's
  0c.
- Exposing `ecs::World` to downstream users — 0e (post-flip).
- Scenario files / fidelity — Phase 1.
- Adding YAML `Configure(YAML::Node)` — 0d.
- Shrinking or re-using archetype chunks — deferred.

## 3. Architecture

```
       ┌─────────────────────────────────────────────┐
       │   Existing public API (unchanged signatures)│
       │   EntityComponentManager::*                 │
       └───────────┬─────────────────────┬───────────┘
                   │                     │
      GZ_SIM_ARCHETYPE_ECM=OFF    GZ_SIM_ARCHETYPE_ECM=ON
                   │                     │
                   ▼                     ▼
        ┌──────────────────┐   ┌──────────────────────┐
        │  LegacyImpl      │   │  ArchetypeImpl       │
        │  (current code)  │   │  → ecs::World (0a)   │
        └──────────────────┘   └──────────────────────┘
```

Selection is a compile-time `using Impl = ...` typedef inside the ECM's
PIMPL. No virtual dispatch, no runtime cost. The legacy `dataPtr` struct
moves into `LegacyImpl`; a new `ArchetypeImpl` struct wraps `ecs::World` + a
few translation caches.

We pick compile-time over runtime-switch because:
- No ABI penalty.
- Forces us to keep the facade honest — there's no "escape hatch" that
  inspects the backend type.
- 0e's flip is a one-liner in CMake once CI is green in both modes.

## 4. API surface mapping

Every public method mapped explicitly. Methods marked ✓ are trivial
one-to-one; others are discussed in later sections.

### 4.1 Entity lifecycle
| ECM method | Archetype backend |
|---|---|
| `CreateEntity()` ✓ | `World::CreateEntity()` |
| `RequestRemoveEntity(e, recursive)` | push into removal queue, applied in `ProcessRemoveEntityRequests` |
| `ProcessRemoveEntityRequests()` | drain queue → `World::Destroy` for each, then `Commit` |
| `HasEntity(e)` ✓ | `World::Alive(e)` |
| `EntityByComponents(...)` | scan via a transient `Query` for the component tuple |
| `SetParentEntity(c, p)` | add/replace `components::ParentEntity{p}` |
| `PinEntity(e, recursive)` / `UnpinEntity` | see §8 (levels) |
| `ChildrenByComponents(parent, ...)` | iterate children of parent, filter by query |

### 4.2 Component lifecycle
| ECM method | Archetype backend |
|---|---|
| `CreateComponent<T>(e, data)` | `World::Add(e, T{data})` — **deferred** (§6) |
| `RemoveComponent(e, typeId)` | `World::Remove(e, typeId)` — deferred |
| `Component<T>(e)` / `Component<T>(e) const` | `World::Component<T>(e)` — pointer valid until next `Commit` (§6) |
| `ComponentData<T>(e)` ✓ | convenience wrapper, unchanged |
| `SetComponentData<T>(e, data)` | `World::Component<T>(e)` + assign — triggers dirty bit |
| `ComponentTypes(e)` | read archetype of `e`, return its type set |
| `HasComponentType(typeId)` ✓ | query `ComponentTypeRegistry::IsRegistered` |
| `EntityHasComponentType(e, typeId)` | check archetype's type set |

### 4.3 Iteration
| ECM method | Archetype backend |
|---|---|
| `Each<T...>(fn)` ✓ | `World::Each<T...>(fn)` |
| `EachNew<T...>(fn)` | iterate chunks whose "newly-created" bit is set, see §7 |
| `EachRemoved<T...>(fn)` | ECM tombstones persist through one full step — preserved via a per-phase "pending-removal" archetype-list snapshot |
| `EachNoCopy<T...>` | same as `Each`; no-copy is a legacy perf hack no longer meaningful |
| `ForEach` (static helper) ✓ | unchanged |

### 4.4 State / serialization
| ECM method | Archetype backend |
|---|---|
| `State(...)` / `State(stateMsg)` | iterate archetypes, emit component messages per column — see §9 |
| `SetState(stateMsg)` | apply via a single command buffer, one `Commit` at end |
| `Changed(...)`, `ChangedState(...)` | walk dirty bitsets to build diffs |
| `SetChanged(e, typeId, state)` | explicit dirty-bit poke |
| `ResetTo(other)` | clone `ecs::World` by column-wise memcpy (trivially relocatable types) + placement move (non-trivial) |
| `CopyFrom(other)` | delegate to `ResetTo` semantics |
| `EntityComponentManagerDiff` | replaced by archetype-level diff: emit changed rows per archetype. Wire-compatible with existing diff struct. |

### 4.5 Views
Views (`BaseView` etc.) are the legacy ECM's query cache. The archetype
backend replaces this with `ecs::Query` cached per callsite. The public
`RebuildViews()` method becomes a no-op under the archetype backend (queries
self-invalidate on archetype-graph edits).

`LockAddingEntitiesToViews(bool)` is a synchronization hack that matters
only for the legacy path. Under archetypes it's also a no-op — parallel
`EachParallel` never mutates the archetype graph (mutations defer to
`Commit`).

### 4.6 Change tracking
| ECM method | Archetype backend |
|---|---|
| `ClearNewlyCreatedEntities()` | clear the "newly-created" archetype side-list |
| `ClearRemovedComponents()` | reclaim tombstoned rows, clear removed-components side-list |
| `SetAllComponentsUnchanged()` | zero the dirty-bit arena |

## 5. Commit scheduling

The legacy ECM applies mutations immediately. The archetype backend defers
them. The facade hides this from callers by inserting `Commit()` calls at
the points the legacy semantics materialized state changes:

- After `ProcessRemoveEntityRequests()`.
- After `SetState(...)`.
- After `ResetTo(...)`.
- At the end of each system phase — we hook this via a new
  `EntityComponentManager::Commit()` method called by `SimulationRunner`
  between PreUpdate/Update/PostUpdate (once each per step). See §8.
- Before `Each*` variants that advertise "sees all pending mutations"
  (currently only `State()` and `ChangedState()`).

This matches the locked deferred-mutation contract without the callers
knowing they're on deferred semantics, except for the pointer-validity
shift (§6).

Performance note: the 3× per-step Commit cost is negligible in practice
because each Commit drains a small command buffer; the expensive path is
the actual system iteration.

## 6. Pointer validity

**Promise under 0b**: a pointer returned by `Component<T>(e)` is valid for
the **duration of the current system phase**. Invalidated by the next
`Commit()`, which the facade runs at phase boundaries.

In practice this means:
- Code that obtains a pointer and uses it within a single
  PreUpdate/Update/PostUpdate callback — **safe** (legacy behavior preserved).
- Code that stores an ECM pointer across phases — **broken**. This was
  already technically undefined under the legacy ECM (any intervening
  mutation could invalidate), but may have worked in practice.

0c includes a grep + manual audit of in-tree systems for cross-phase
pointer holding. External plugin migration notes will be in `Migration.md`.

## 7. Change tracking — preserving `EachNew` / `EachRemoved` / `EachChanged`

The legacy ECM tracks three change categories:
- **New**: entities/components created this step (before
  `ClearNewlyCreatedEntities`).
- **Removed**: entities/components tombstoned this step (before
  `ClearRemovedComponents`).
- **Modified**: components whose value changed this step (before
  `SetAllComponentsUnchanged`).

Archetype-backend equivalents:

**New**: each archetype keeps a `newly_created_rows` bitset per chunk. On
`World::Add(e, T)` that transitions `e` into archetype `A`, the destination
row's bit is set in `A.newly_created`. `ClearNewlyCreatedEntities()` zeroes
all such bitsets.

**Removed**: `World::Remove` on entity or component appends the removal
descriptor `(ArchetypeId old, ComponentTypeId t, saved_value)` to a
per-step `removal_log`. `EachRemoved<T>` walks the log. The saved value is
needed because callers expect to see the component's pre-removal state (used
by scene_broadcaster, logging). Cleared by `ClearRemovedComponents()`.

**Modified**: the dirty-bit arena described in 0a §5.8. `EachChanged<T>`
iterates chunks and yields rows with the column's dirty bit set.

These three together exactly cover the existing API surface. The legacy
`_changedComponents` / `_newlyCreatedEntities` sets are replaced by these
structured side-tables.

## 8. Levels, parents, pinning

### 8.1 Level pinning

`LevelManager` calls `PinEntity(e, recursive)` to prevent an entity from
being unloaded even when no performer is near it. Under archetypes, pinning
is a `components::LevelPin{}` tag applied to the entity (and children if
recursive). LevelManager already handles the bookkeeping; it just needs to
use the tag consistently under both backends.

### 8.2 Level load/unload

Level loading creates a bunch of entities at once; level unloading removes
them. Under archetypes:
- **Load**: executed as a batch of queued commands → one `Commit` at end.
  Cheaper than the legacy path because related entities tend to share
  archetypes (same component set) and land in the same chunks.
- **Unload**: batched destroy → one `Commit`. Archetype rows are
  swap-with-last; archetypes with zero remaining rows stay allocated (0a
  doesn't shrink).

### 8.3 Parent/child

`components::ParentEntity{parent}` — already an existing component, no
change. `ChildrenByComponents` scans the archetypes matching the filter,
reads the ParentEntity column, filters by parent. Could be accelerated by
a parent-to-children index, but that's deferred — first-pass measurement
will tell us if it matters.

## 9. Serialization

`State(msgs::SerializedStateMap &msg)` currently walks the per-entity
component map and emits one `SerializedComponent` per component. Under
archetypes we walk chunks column-by-column:

```
for each archetype A:
  for each chunk C in A.chunks:
    Entity* es = C.entities()
    for each type T in A.types:
      ComponentT* col = C.column<T>()
      for row in 0..C.count:
        if (filter allows es[row] and T):
          emit SerializedComponent(es[row], T, col[row])
```

Output wire format is unchanged. Consumers (`scene_broadcaster`, network
secondaries, state log recorder) see no difference.

`ChangedState()` uses dirty bitsets to skip unchanged rows — natural fit,
cheaper than today.

`SetState(msgs::SerializedStateMap)` buffers all component creations/removals
into a single command buffer, applies them in one `Commit`. This is
noticeably faster than the legacy path on level-load-sized messages.

Tombstone persistence: `msgs::SerializedState` distinguishes newly-removed
components (needs `ComponentState::OneTimeChange`) from steady state.
`removal_log` (§7) carries the info needed to drive this. Equivalent
behavior is a test gate.

## 10. Integration touchpoints in gz-sim

Files that import ECM internals and may need small edits even in 0b:

- [src/SimulationRunner.cc](../../src/SimulationRunner.cc) — add `Commit`
  calls at phase boundaries (PreUpdate→Update, Update→PostUpdate, end-of-
  step). Under legacy backend these are no-ops.
- [src/SdfEntityCreator.cc](../../src/SdfEntityCreator.cc) — entity-creation
  hot path, heavy ECM usage. Verify all pointer uses are single-phase. Expected zero code change; just audit.
- [src/LevelManager.cc](../../src/LevelManager.cc) — load/unload batching
  already exists; confirm it plays nicely with deferred semantics. Expected
  zero code change.
- [src/network/NetworkManagerPrimary.cc](../../src/network/NetworkManagerPrimary.cc)
  and `NetworkManagerSecondary.cc` — `SetState`/`State` users, validated
  via round-trip tests.
- [src/SystemManager.cc](../../src/SystemManager.cc) — invokes system
  callbacks; wiring `Commit` at the right boundaries.

No API changes to systems themselves in 0b. System ports are 0c.

## 11. Build system

CMake:
```cmake
option(GZ_SIM_ARCHETYPE_ECM
  "Use archetype-based ECS backend (Phase 0b)." OFF)
if (GZ_SIM_ARCHETYPE_ECM)
  target_compile_definitions(gz-sim PRIVATE GZ_SIM_USE_ARCHETYPE_ECM=1)
  target_link_libraries(gz-sim PUBLIC gz-sim-ecs)
endif()
```

The `gz-sim-ecs` target was created in 0a. 0b links it into `libgz-sim`
conditionally. Both source trees (`src/legacy_ecm/` and the archetype
delegations) are compiled — each behind an `#if GZ_SIM_USE_ARCHETYPE_ECM`.

CI matrix adds a second row running with the flag ON. This doubles build
+ test time for 0b–0d but is worth it. Dropped after 0e.

## 12. Test plan

- **Existing unit test suites** (`EntityComponentManager_TEST`,
  `SimulationRunner_TEST`, `LevelManager_TEST`, `SdfEntityCreator_TEST`,
  `ComponentFactory_TEST`, `BaseView_TEST`) — all must pass under both
  backends. Any test relying on legacy-only behavior is rewritten to the
  documented contract.
- **New**: `EntityComponentManager_ArchetypeParity_TEST.cc` — parameterized
  over backends, runs the same scripted mutation sequence against each and
  diffs state/view outputs. Catches semantic drift.
- **Serialization round-trip**: build a world with all in-tree component
  types populated, serialize, deserialize into a fresh ECM, serialize
  again, byte-compare messages.
- **Pointer-validity audit**: new clang-tidy check or a small custom AST
  matcher that flags `Component<T>` return values stored into fields /
  statics (across-phase retention). Findings reviewed manually.
- **Network primary/secondary integration tests** — run existing
  `network_handshake` and related tests under both backends.
- **Benchmarks** — the 0a benches run inside gz-sim now with the flag on,
  not just the standalone library. Confirm no integration regression.

## 13. Migration notes (for Migration.md)

In 0b nothing changes for downstream users (default is OFF). In 0e,
`Migration.md` will note:
- Pointer returned by `Component<T>` valid for current system phase only.
- `EntityByComponents`, `EachNew`, `EachRemoved` semantics unchanged but
  faster.
- Views API (`BaseView` direct usage) discouraged; construct queries via
  the ECM's `Each` variants.
- ABI break: recompile required.

## 14. Risks

**R1 — undocumented legacy semantics.** Some downstream code may rely on
behavior that isn't in the header docs (e.g., iteration order, specific
timing of `ClearNewly*`). The parity test + external beta on the fork
shakes these out.

**R2 — performance regression during 0b.** The facade might be slower than
legacy if `Commit` interval is too short or if a hot path hits a
non-optimized translation. Gate: the 0a benches as integrated tests must
still show the ≥ 3× multi-robot improvement — if not, fix before 0c.

**R3 — serialization drift.** Emitting columns in archetype order may
produce different but equivalent messages. Consumers should be
order-independent but may not be. Addressed by the round-trip byte-compare
test; if it fails, we sort emission by (entity_id, type_id) for
compatibility.

**R4 — pointer-validity audit misses a case.** Partial mitigation: the
debug generation-check in 0a fires an assert in tests. If a system
triggers it, fix is mechanical.

## 15. Milestones

| Week | Deliverable |
|---|---|
| 1 | `EntityComponentManager` internal split: `Impl` typedef, `LegacyImpl` wrapper on existing code, build passes with flag OFF. |
| 2 | `ArchetypeImpl`: entity lifecycle, component lifecycle, `Each<T...>`. ECM_TEST builds with flag ON. |
| 3 | `EachNew` / `EachRemoved` / `EachChanged`, dirty bitset wiring, change-tracking tests pass. |
| 4 | `State` / `SetState` / `ChangedState` serialization parity. Round-trip test green. |
| 5 | `Commit` hooks in `SimulationRunner`. Full unit test suite green under both backends. |
| 6 | Integration tests: levels, network primary/secondary, scene broadcaster. Fix any last semantic drift. |
| 7 | Parity benchmarks in gz-sim; confirm 0a gains survive the facade. |
| 8 | Documentation, Migration.md notes drafted (for 0e), 0b tagged complete. |

Two-week buffer acceptable; three weeks triggers a scope review — most
likely candidate to drop is the serialization-parity polish, which can
slip into 0c.

## 16. What unblocks after 0b

- **0c** can port in-tree systems to use deferred mutations explicitly and
  drop any legacy pointer-across-phase usage found in the audit.
- **0d** (YAML Configure) is independent and can run in parallel with 0c.
- **Phase 1** (scenario files) is unblocked — the facade is stable enough
  to parse scenarios and drive entity creation against it.
