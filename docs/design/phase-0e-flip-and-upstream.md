# Phase 0e — Flip the Default, Remove Legacy, Prep Upstream

Status: Draft
Owner: Nathan Koenig
Scope: Phase 0e of the Adjustable-Fidelity initiative
Depends on: [0a](phase-0a-archetype-ecm.md), [0b](phase-0b-ecm-integration.md), [0c](phase-0c-system-port.md), [0d](phase-0d-yaml-configure.md)
Blocks: Phase 1 (scenarios — but can proceed once archetype backend is the
default)

## 1. Context

0a–0d delivered the archetype core, the facade, system compatibility, and
the YAML configure path, all behind the `GZ_SIM_ARCHETYPE_ECM=ON` flag.
0e retires the legacy backend, removes the flag, commits to the new ABI
on the fork, and opens the upstream conversation.

## 2. Goals / Non-goals

Goals:
- Flip the CMake default to archetype backend.
- Delete the legacy ECM storage code path.
- Update `Migration.md` with the full ABI break rationale and migration
  recipes.
- Open the upstream RFC (`gazebosim/gz-sim` Discussions + RFC PR) seeking
  maintainer alignment.
- Tag a fork release (`adjustable-fidelity-0e`) as the foundation for
  Phase 1.

Non-goals:
- Any new functionality.
- Upstream merge itself — that's a follow-on that depends on maintainer
  feedback. 0e completes when the RFC is **open**, not merged.
- Python bindings revisit (tracked separately).

## 3. Gate to start 0e

All of:
- 0a–0d test suites green in archetype mode continuously for ≥ 2 weeks.
- No P0/P1 archetype-specific bugs in the issue tracker.
- Benchmark history shows no regression vs. the 0a numbers.
- Serialization parity tests (state round-trip) green for 2 consecutive
  weeks.

If these aren't met, 0e waits. Bias is toward delaying the flip rather than
flipping on an uncertain foundation — once removed, legacy is gone.

## 4. Flip mechanics

### 4.1 CMake

```cmake
# Before (0b-0d):
option(GZ_SIM_ARCHETYPE_ECM "Use archetype-based ECS backend." OFF)

# After (0e):
# (option removed; archetype is the only backend)
```

All `#if GZ_SIM_USE_ARCHETYPE_ECM` guards deleted; legacy branches removed.

### 4.2 Files deleted

- `src/legacy_ecm/` directory and contents.
- Legacy test fixtures that only exercised pre-archetype semantics.
- The dual-CI matrix row.

### 4.3 Files kept but simplified

- `EntityComponentManager.cc`: now a thin forwarder to `ecs::World`. Still
  a PIMPL (for ABI insulation from future changes), but the `Impl` is
  no longer a typedef — it's the archetype implementation directly.
- `Migration.md`: rewritten with the formal ABI/semantic break section.

## 5. Migration notes published with 0e

`Migration.md` additions:

```
## Gazebo <next> — Archetype ECS

### ABI break (recompile required)

This release replaces the EntityComponentManager's internal storage with
an archetype-based ECS. All `libgz-sim` consumers must recompile.

### Public API changes

No public function signature changed. Semantic changes:

- `ecm.Component<T>(entity)` return values are valid for the current
  system phase only. The pointer is invalidated when the runner calls
  `Commit()` at the next phase boundary (end of PreUpdate, Update,
  PostUpdate).
- `CreateComponent`, `RemoveComponent`, `SetParentEntity`, and
  `RequestRemoveEntity` now defer. The mutation is applied at the next
  Commit (same boundaries). Subsequent lookups of the same component
  within the same phase see the *pre-mutation* state.
- `RebuildViews()` is a no-op (retained for compatibility).
- Iteration order across `Each<T...>` is not defined (it wasn't before
  either; now we're explicit).

### Migration recipe for downstream plugins

- Stop caching `Component<T>` pointers in system members. Acquire in the
  callback, use locally.
- If you mutate and then read the mutation in the same phase, combine
  the two (`CreateComponent(e, T{value})` instead of create-then-set), or
  split the read into a later phase.
- Remove any `RebuildViews()` calls — they're redundant.

### Performance

- `Each<T...>` iteration is 3–10× faster depending on component set size.
- Level-load and batch-spawn are noticeably faster (single-Commit amortization).
- Per-entity lookup is roughly unchanged (hash table indirection replaces
  unordered_map indirection).

### Migration tool

(Currently no automated tool. See "audit-checklist.md" for a manual
checklist we used on our in-tree systems.)
```

## 6. Upstream RFC

Open a formal RFC in `gazebosim/gz-sim` Discussions, followed by an RFC
PR that contains:

- **Motivation** — adjustable-fidelity goals, scaling to 100+ AMRs.
- **Proposal** — archetype ECS; summary from 0a + 0b.
- **Alternatives considered** — adopting flecs/EnTT (rejected, duplication
  with existing gz-sim concepts); pool-per-type without full archetypes
  (rejected as half-measure after perf measurements — include numbers).
- **Migration impact** — quantified from our fork's audit: how many
  in-tree systems needed changes, how many were mechanical, where humans
  had to think.
- **Compatibility** — source-level unchanged public API, ABI break
  justified by perf + clarity of contract.
- **Rollout plan** — same phased structure we followed; maintainers can
  replay it upstream.
- **Benchmarks** — 0a numbers + real-world warehouse scene.

The RFC is opened at end of 0e. Maintainer discussion may iterate the
design before upstream merge lands (separate, unscoped timeline). Our
fork continues on Phase 1+ in parallel.

❓ **Q26.** Who leads the upstream conversation — you, me (as the
engineer on the fork), or a broader Kuka delegation? Relevant because
the RFC author drives the review cadence and risks getting blocked on
maintainer responsiveness. Default: you, with me drafting and attending
technical discussions.

❓ **Q27.** Upstream wants to land pieces incrementally or as a single
big-bang? Our phase structure is incremental, but the ABI break is by
nature all-at-once. Worth asking maintainers what they'd prefer before
committing. I'd propose to them: land ECM facade + archetype backend
behind a flag first (mirrors 0b), then flip the default in a subsequent
release.

## 7. Risks

**R1 — A latent legacy-semantics bug discovered after flip.** Mitigated
by the 2-week stability gate and the parity test suite. If one slips,
fork `hotfix` release with the legacy path restored temporarily is the
escape hatch — but only for 0e, not indefinitely.

**R2 — Upstream maintainers reject the approach.** Then we carry the
fork indefinitely; not desirable but not blocking our own goals. We
keep rebasing on upstream `main` to minimize merge drift.

**R3 — Downstream consumers (ROS integrations, closed Kuka code,
community plugins) break.** Expected and documented. The Migration.md
recipes and the 2-week gate should catch most in our own tree. Community
pain happens at upstream merge time, not at our fork flip.

## 8. Milestones

| Week | Deliverable |
|---|---|
| 1 | Stability-gate verification; benchmark regression history reviewed. |
| 2 | Legacy code deleted; CMake flag removed; CI simplified; fork tag cut. |
| 3 | Migration.md final; upstream RFC drafted; RFC opened. |

Three weeks. Week 3 is mostly writing and coordinating; actual code
changes concentrate in week 2.

## 9. Exit criteria

- Archetype backend is the only backend.
- `Migration.md` published.
- Upstream RFC open with initial maintainer responses received.
- Fork tag `adjustable-fidelity-0e` cut.
- Phase 1 unblocked.
