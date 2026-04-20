# Phase 0c — Port In-Tree Systems to Deferred-Mutation Semantics

Status: Draft
Owner: Nathan Koenig
Scope: Phase 0c of the Adjustable-Fidelity initiative
Depends on: [0a](phase-0a-archetype-ecm.md), [0b](phase-0b-ecm-integration.md)
Blocks: 0e (flip default)

## 1. Context

Phase 0b established the archetype-backed ECM behind a compile-time flag.
The facade preserves semantics **except** for pointer-validity across phase
boundaries. Phase 0c audits every in-tree system for latent reliance on
legacy semantics and fixes what's found.

This is operational work, not new design. The primary deliverable is a
**clean CI run** in archetype mode across the full gz-sim test suite.

## 2. Goals / Non-goals

Goals:
- Audit every `.cc` under `gz-sim/src/systems/` for:
  - ECM pointers retained across phase boundaries.
  - `CreateComponent` / `RemoveComponent` calls followed by immediate reads
    of the mutation within the same phase.
  - Reliance on `RebuildViews()`-style explicit view refresh.
- Mechanical fixes to conform to deferred semantics.
- Green CI in archetype mode.
- `Migration.md` entries for any observable semantic change.

Non-goals:
- Refactoring systems for performance (wait for Phase 5).
- New features or new plugins.
- Public API changes beyond the ECM pointer-validity contract.
- Port to the native YAML Configure path (that's 0d).

## 3. Audit methodology

### 3.1 Automated pass

Two clang-based checks run in CI:

1. **`component-ptr-crossphase`** — custom clang-tidy: flags any
   `ComponentT *` / `ComponentT &` stored in a field of a `System` subclass.
   Pointers must be locals within a single callback.

2. **`mutate-then-read`** — AST matcher that flags the pattern:
   ```cpp
   ecm.CreateComponent(e, T{});        // or RemoveComponent, SetParentEntity
   ecm.Component<T>(e);                // same phase, same entity
   ```
   False positives are acceptable; human reviews the list.

Both checks run in `ctest -L lint` and block merges on regression after 0c
completes.

### 3.2 Manual pass

For each system in `gz-sim/src/systems/`:
1. Read the `Configure`, `PreUpdate`, `Update`, `PostUpdate` methods.
2. Trace ECM interactions: where pointers come from, how long they're used.
3. Check for cross-phase state: members holding `Entity` handles are fine;
   members holding `ComponentT *` are not.

Audit checklist lives in `gz-sim/docs/design/0c-audit-checklist.md`
(produced as work artifact, one row per system).

## 4. Expected fix categories

Based on spot-reading of the systems, these patterns are likely candidates
for adjustment:

### 4.1 Cached component pointers in members
**Before**:
```cpp
class DiffDrive::Impl {
  components::JointVelocityCmd *leftCmd_ = nullptr;  // cached across phases
};
void DiffDrive::PreUpdate(...) {
  if (!impl->leftCmd_)
    impl->leftCmd_ = ecm.Component<JointVelocityCmd>(impl->leftJoint_);
  impl->leftCmd_->Data() = targetVel;
}
```
**After**:
```cpp
void DiffDrive::PreUpdate(...) {
  auto *leftCmd = ecm.Component<JointVelocityCmd>(impl->leftJoint_);
  leftCmd->Data() = targetVel;
}
```
The lookup is cheap under archetypes (direct `EntityIndex` hit then column
index). Caching adds bug surface with no win.

### 4.2 Create-then-read within the same phase
**Before**:
```cpp
ecm.CreateComponent(e, components::WorldPose{});
auto *p = ecm.Component<components::WorldPose>(e);
p->Data() = initialPose;
```
**After** — combine:
```cpp
ecm.CreateComponent(e, components::WorldPose{initialPose});
```
Or, if the value genuinely must be computed after the creation:
```cpp
ecm.SetComponentData<components::WorldPose>(e, initialPose);   // queued
// Do not read back before next Commit.
```

### 4.3 Lazy component creation during iteration
**Before**: `Each<A>` that calls `CreateComponent<B>` on matched entities,
then iterates `Each<A, B>` later in the same phase and expects the new B's
to appear.
**After**: split into two phases, or pre-create B's in a prior phase, or
rely on an explicit `Commit()` (facade exposes it but in-tree systems
should avoid — let the runner-driven boundary handle it).

### 4.4 Manual `RebuildViews()`
Archetype backend has no views in the legacy sense — `RebuildViews` is a
no-op. Calls can be removed. Grep will find them.

## 5. System-by-system status tracking

A checklist file `docs/design/0c-audit-checklist.md` lists every system
under `src/systems/` with columns:
- `audited` (date + reviewer)
- `violations` (count)
- `fix_pr` (PR link)
- `archetype_tests_pass` (date)

Kept current throughout 0c. Gate for 0c completion: every row green.

## 6. Contributed systems outside `src/systems/`

Some functionality lives in adjacent libraries or is registered from
application code (e.g., `scene_broadcaster`, `sensors` aggregator, the
physics system itself). These get audited too:

- `src/systems/physics/Physics.cc` — heavy ECM user, audit with extra
  care. No behavior change expected but the volume of interactions is
  high.
- `src/systems/scene_broadcaster/SceneBroadcaster.cc` — serialization
  path; covered by the 0b serialization parity tests.
- `src/systems/sensors/Sensors.cc` — rendering orchestration; watch for
  cross-phase pointer holding around the render thread.

❓ **Q22.** Do you want 0c's audit artifact (`0c-audit-checklist.md`) kept
in-tree long-term as a review reference, or deleted after 0e flips the
default? I'd keep it — future systems can use it as a review template.

## 7. Regression test strategy

The existing gz-sim test suite is the primary regression harness. CI
matrix already runs both backends in 0b–0d. Additional 0c-specific tests:

- **Pointer-lifetime asserts**: in debug builds, the archetype ECM's
  generation-check asserts on stale pointer deref. 0c tests run with these
  asserts on and are required to not trip them.
- **"No-op phase" test**: run one step of each system on an empty or
  trivially-populated world, confirm no crash and no unexpected mutations.
- **Fuzz**: a randomized world generator + random-system-ordering test
  that runs 1000 steps under both backends, byte-compares state at the
  end. Catches subtle sequencing differences.

## 8. Risks

**R1 — Hidden legacy-semantic reliance in third-party plugins.** Out of
scope for 0c (those aren't in-tree), but flagged for 0e's Migration.md.

**R2 — Fix that changes observable behavior.** Possible if a system
relied on a mutation being visible mid-phase for a subsequent lookup;
moving the lookup across a Commit boundary changes timing. If detected
by a test, the system is redesigned — not the ECM contract.

**R3 — Audit misses a case.** Debug generation-check catches the worst
cases at test time. Release builds elide the check, so production users
on 0e are protected only if the test suite exercised the path. Expand
tests on any finding.

❓ **Q23.** If 0c audit finds a system whose current behavior relies on
seeing a mid-phase mutation, and the fix is non-trivial (e.g., requires
splitting into two passes or adding a new component), do you want me to
(a) do the refactor as part of 0c, (b) leave the system's test skipped
in archetype mode and fix it in 0e, or (c) block 0c until the system is
reworked? My default: (a) for small fixes, (b) for heavy ones with a
filed follow-up ticket.

## 9. Milestones

| Week | Deliverable |
|---|---|
| 1 | Audit infrastructure (clang-tidy check, AST matcher, checklist file). Audit first 10 systems. |
| 2 | Audit next 20 systems. Fix PRs landing continuously. |
| 3 | Remaining systems audited. All mechanical fixes merged. |

Three weeks. Can compress if audit is quieter than expected; no padding.

## 10. Exit criteria

- 100% of in-tree systems audited (checklist rows green).
- Full gz-sim test suite green in archetype mode.
- Clang-tidy check `component-ptr-crossphase` wired into CI.
- `Migration.md` updated with pointer-validity contract.
- No open P0/P1 issues referencing archetype-mode failures.

When those five land, 0c is done and 0d / 0e unblock.
