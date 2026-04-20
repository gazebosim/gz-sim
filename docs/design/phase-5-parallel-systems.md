# Phase 5 — Parallel PreUpdate / Update via System Dependency Graph

Status: Draft
Owner: Nathan Koenig
Scope: Phase 5 of the Adjustable-Fidelity initiative
Depends on: 0a–0e (archetype ECM), [Phase 1](phase-1-scenarios.md)
(scenarios), [Phase 2](phase-2-kinematic-tier.md) (kinematic system
exists and is hot path at scale)
Blocks: nothing critical; the **final 5× RTF gate** on 100 AMRs is met
here.

## 1. Context

Today only PostUpdate runs in parallel, via a barrier
([SimulationRunner.cc:552](../../src/SimulationRunner.cc#L552)). PreUpdate
and Update are serial. At 100 AMRs, `kinematic_motion`'s PreUpdate and
`Physics.cc`'s Update dominate wall-clock; serial execution leaves cores
idle.

Phase 5 generalizes: systems declare their read/write component sets, a
dependency graph is built, and a work-stealing scheduler executes
non-conflicting systems in parallel during PreUpdate and Update. This is
the final push to ≥ 5× RTF on `warehouse_100_amr_fast.yaml`.

## 2. Goals / Non-goals

Goals:
- System-side declaration API for read/write component types (and entity
  sets, optionally).
- Static dependency-graph construction at SimulationRunner startup.
- Work-stealing scheduler for PreUpdate and Update phases.
- Deterministic mode (`runtime.deterministic: true`) that falls back to
  serial execution.
- `warehouse_100_amr_fast.yaml` hits ≥ 5× RTF gate on reference box.
- No correctness regressions in any existing test.

Non-goals:
- Runtime system reordering based on measured cost (deferred).
- GPU offload of systems (deferred).
- Fine-grained intra-system parallelism — systems already use
  `EachParallel` from 0a for that. Phase 5 is about **between**-system
  parallelism.
- Parallel PostUpdate rework — existing barrier-based approach kept.

## 3. Architecture

```
           SimulationRunner
                  │
                  ▼
          SystemScheduler  (new)
                  │
                  ├─── Reads  declarations   from each System's metadata
                  ├─── Builds DAG            at Configure-time
                  └─── Produces execution plan per phase
                            │
                            ▼
                   WorkerThreadPool  (existing)
                    │        │        │       │
                  Worker1  Worker2  Worker3  Worker4
                    │        │        │       │
                 System A │ System C │ System F│ System H
                 System B │ System D │ System G│
                          │ System E │          │
```

Each phase (PreUpdate, Update) gets its own DAG. The scheduler dispatches
independent systems to workers. A system starts when all its predecessors
have completed.

## 4. Declaration API

### 4.1 System metadata

Systems today register interfaces via inheritance. Phase 5 adds an
optional **metadata** hook:

```cpp
class ISystemConfigure { ... };        // unchanged
class ISystemPreUpdate { ... };        // unchanged
// New:
class ISystemMetadata {
  virtual SystemAccess DescribeAccess(const Entity &_entity,
                                      const EntityComponentManager &_ecm) const = 0;
};

struct SystemAccess {
  // Component types this system reads:
  std::vector<ComponentTypeId> reads;
  // Component types this system writes:
  std::vector<ComponentTypeId> writes;
  // Exclusive access needed (e.g. for state serialization)?
  bool exclusive = false;
  // Optional: per-entity specialization (future)
  // std::optional<EntityFilter> filter;
};
```

Systems that don't implement `ISystemMetadata` default to **exclusive
access** — they run serially with a warning. This is the opt-in gate
for parallel execution; no system is forced.

### 4.2 Migration for in-tree systems

Porting a system is small:
```cpp
SystemAccess KinematicMotion::DescribeAccess(...) const {
  return {
    .reads  = { types::LinearVelocityCmd, types::AngularVelocityCmd },
    .writes = { types::Pose, types::JointPosition, types::JointVelocity },
  };
}
```

All in-tree systems are ported as part of Phase 5. Out-of-tree systems
that don't opt in still work (serially); migration docs explain the
pattern.

## 5. Dependency graph construction

### 5.1 Conflict rules

Two systems S1 and S2 **conflict** (one must run before the other) if:
- S1.writes ∩ S2.writes ≠ ∅ (write-write), or
- S1.writes ∩ S2.reads  ≠ ∅ (write-read in either direction), or
- Either is `exclusive = true`.

Non-conflicting systems may run in parallel.

### 5.2 Ordering constraints

Gz-sim already orders systems by a user-specified priority. Phase 5
preserves priorities:
- Within a priority group, systems execute in parallel where possible.
- Priority groups are barriers: all systems in priority N complete before
  any system in priority N-1 starts.

This gives users explicit control (priority) without killing parallelism
(non-conflicting systems within a group parallelize).

### 5.3 Graph structure

Per phase, a DAG:
- **Nodes**: systems in the phase.
- **Edges**: conflicts + priority-group boundaries.
- **Topological levels**: used by the scheduler to dispatch waves.

Built once at Configure-time; rebuilt only when systems are added/removed
at runtime (rare — typically only during level loading).

## 6. Scheduler

### 6.1 Work-stealing approach

Standard work-stealing: each worker has a local deque; idle workers steal
from others' tails. Gz-sim has `gz::common::WorkerThreadPool`; Phase 5
extends it with a `RunDAG(nodes, edges)` helper if not already present.

### 6.2 Execution

```
RunPhase(phase):
  counters[node] := in_degree(node)
  ready_queue := nodes with in_degree == 0
  while any node unfinished:
    dispatch nodes from ready_queue to workers
    when worker finishes node N:
      for succ in successors(N):
        if --counters[succ] == 0:
          ready_queue.push(succ)
  wait for all workers idle
```

### 6.3 Python GIL

Python-implemented systems (via `python_system_loader`) hold the GIL;
parallel execution of multiple Python systems deadlocks on it. The
scheduler treats Python systems as `exclusive = true` and serializes
them. Mixed Python+C++ scenes parallelize the C++ portion.

## 7. Deferred mutations + parallel execution

Archetype ECM's deferred-mutation contract (from 0a) integrates cleanly:
- Each worker owns a thread-local command buffer.
- All mutations in a phase defer.
- `Commit()` runs once after the phase completes, on the main thread
  (no parallelism during commit).

This is exactly the design 0a baked in. Phase 5 activates it for
PreUpdate/Update where it was previously unused.

## 8. Deterministic mode

`runtime.deterministic: true` (scenario-level) forces:
- Serial execution of all systems in insertion order (current behavior).
- `EachParallel` degrades to serial.
- Worker pool sized to 1 (or skipped).

This preserves bit-for-bit reproducibility for users who need it
(accurate-tier validation, replayable incident reproduction).

Overhead of determinism: ~1–2% scheduler code executes regardless. The
scheduler branches on the flag at phase entry.

❓ **Q37.** Default for `deterministic`? I propose `false` at the runtime
level (maximize throughput) and override per-profile: `accurate` profile
sets `deterministic: true` by default. Users who want an accurate run
that parallelizes can flip the per-profile default, accepting minor
numerical noise.

## 9. Scheduler overhead

At small entity counts the scheduler adds per-phase overhead (DAG traversal,
worker dispatch, atomics on counters). Expected: 10–30 μs per phase on
an 8-core system with ~50 systems. Negligible at 100 AMRs; measurable
at trivial-scene unit tests.

Mitigation: if `systems_in_phase <= 4`, fall back to serial execution
inline. Threshold tunable; default 4 chosen from back-of-envelope.

## 10. Migration impact on in-tree systems

A script sweeps `src/systems/` and produces migration commits. Expected
pattern for most systems: add `DescribeAccess` method, list components
touched, done. For the handful of systems doing exotic things (e.g.,
`scene_broadcaster` serializes full state), mark `exclusive = true`.

Priority mapping: existing `Priority` ordering is preserved verbatim;
no re-ordering of in-tree systems.

❓ **Q38.** Do you want a CI lint that fails on any system not declaring
`ISystemMetadata`? Long-term we want 100% adoption; short-term
out-of-tree plugins trip it. Default: warn-only in 0e's first release,
error in the release after.

## 11. Benchmarks

Primary gate: `warehouse_100_amr_fast.yaml`.
- Reference box: 32-core, 64GB.
- 60 s simulated time.
- **Gate: RTF ≥ 5×.**
- Recorded per-phase: PreUpdate time, Update time, PostUpdate time,
  Commit time, scheduler overhead.

Secondary benches:
- `mixed_20_robot.yaml`: expect ≥ 1.5× RTF (up from Phase 3's 1×).
- Small-scene overhead test: 1 robot, 10 systems — confirm scheduler
  overhead under 50 μs/step.

## 12. Risks

**R1 — Incorrect conflict declarations.** A system that writes a
component it forgot to declare causes a data race. Caught by TSan runs
in CI; also by a debug-build check that the ECM's dirty bits after a
parallel phase match the union of declared writes. If they don't, fail
loud.

**R2 — Scheduler starvation.** A heavy system at low in-degree can starve
lighter waiting systems. Work-stealing mitigates. Measured in the bench;
if pathological, switch to priority-based dispatch with cost estimates.

**R3 — Nondeterministic test flakes.** Parallel execution introduces
ordering nondeterminism (e.g., log line order). Tests asserting exact
output order break. Fixed by marking affected tests with
`deterministic: true` scenario or rewriting assertions to be
order-independent.

**R4 — Python plugin dominance.** If users are heavy in Python systems,
parallelization gains evaporate. Mitigation: documentation — Python
systems should be minimal glue; heavy lifting in C++.

❓ **Q39.** For the final 5× RTF gate: is the reference box
(32-core, 64GB) locked, or should I specify a couple of reference
configurations (e.g., 32-core workstation, 8-core laptop)? Laptop users
won't hit 5× on 100 AMRs at 8 cores — closer to 2×. Documenting both is
honest.

## 13. Milestones

| Week | Deliverable |
|---|---|
| 1 | `ISystemMetadata` interface + `SystemAccess` struct landed. In-tree systems annotated. |
| 2 | `SystemScheduler` DAG construction + topological wave dispatch. |
| 3 | Work-stealing runtime via `WorkerThreadPool`; `RunDAG` helper. |
| 4 | Deferred-mutation integration (use 0a's thread-local command buffers in parallel phases). |
| 5 | Deterministic mode; Python-system exclusive handling. |
| 6 | Lints, TSan runs, race-hunting; fix declared-vs-actual access mismatches. |
| 7 | `warehouse_100_amr_fast.yaml` bench; tune thresholds. |
| 8 | Hit 5× gate; docs; Phase 5 tagged. |

Eight weeks. Two-week slip likely on race-hunting (week 6) — experience
says declared access sets are often wrong on first pass.

## 14. Exit criteria

- `SystemScheduler` landed with DAG + work-stealing.
- All in-tree systems declare access.
- `warehouse_100_amr_fast.yaml` ≥ 5× RTF gate passes.
- TSan clean on the full test suite.
- `deterministic` mode preserved bit-for-bit on accurate-tier validation.
