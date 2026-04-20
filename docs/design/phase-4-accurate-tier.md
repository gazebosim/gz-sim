# Phase 4 — Accurate Tier (Bounded-Residual Integration)

Status: Draft
Owner: Nathan Koenig
Scope: Phase 4 of the Adjustable-Fidelity initiative
Depends on: [Phase 3](phase-3-sidecar-engines.md) (accurate tier needs its
own engine slot)
Blocks: nothing on the critical path; Phase 5 proceeds independently.

## 1. Context

The accurate tier is the "correctness guaranteed" mode. Users trading
speed for accuracy (robot-arm simulation, manipulation validation,
contact-rich tasks) run at the accurate tier and expect the simulator to
**fail loudly** if it can't meet a bounded-residual integration
contract — not silently drift.

Phase 4 delivers:
1. A new `gz::physics` feature interface: `StepWithResidual`.
2. Implementations in `dartsim` and `bullet-featherstone`.
3. Adaptive substepping logic in `Physics.cc` for accurate-tier engines.
4. A reference-trajectory validation suite.
5. A policy hook for how to react when the contract can't be met.

## 2. Goals / Non-goals

Goals:
- `StepWithResidual` feature with a stable semantic contract.
- Adaptive substepping: if residual exceeds tolerance, halve `dt` and
  recurse up to `max_substeps`.
- Per-profile configuration of tolerance, max substeps, and failure
  policy.
- Validation suite: reference trajectories for a UR10 pick-and-place and
  a 7-DOF manipulation task.
- CI gate: `ur10_cell_accurate.yaml` scenario passes bounded-residual
  test under both DART and bullet-featherstone.

Non-goals:
- Rewriting DART / Bullet internals — we use their existing integrators,
  we just query their residuals.
- Implicit integration beyond what the engines already offer.
- Mandatory use of the accurate tier — opt-in via scenario.
- Per-step user-visible progress reporting (deferred; CLI logs enough).

## 3. The `StepWithResidual` feature

### 3.1 Semantic contract

```cpp
namespace gz::physics {
  struct StepResult {
    double   residual_norm;   // L2 norm of constraint residuals after step
    uint32_t iterations_used; // solver iterations consumed
    bool     converged;       // residual_norm <= engine's internal tol
  };
  struct StepWithResidual : public virtual FeaturePolicy3d {
    virtual StepResult StepWithResidual(const Identity &_world,
                                        double _dt,
                                        double _residual_tol,
                                        uint32_t _max_iter) = 0;
  };
}
```

Semantics:
- Advance the world by `_dt` using the engine's normal integrator.
- Iterate the constraint solver up to `_max_iter` iterations, aiming for
  `residual_norm ≤ _residual_tol`.
- Return the actual `residual_norm`, iteration count, and convergence
  flag.
- **No fallback**: if the solver doesn't converge, the step is still
  committed — the caller decides what to do. This preserves the
  existing integrator's behavior and gives Physics.cc the freedom to
  substep or reject.

### 3.2 `dartsim` implementation

DART exposes constraint residuals via its `ConstraintSolver`. We wrap:
```cpp
StepResult DartsimEngine::StepWithResidual(...)
{
  auto *solver = world->getConstraintSolver();
  solver->setMaxIteration(_max_iter);
  solver->setResidualTolerance(_residual_tol);
  world->step();
  return { solver->getResidualNorm(),
           solver->getIterations(),
           solver->getResidualNorm() <= _residual_tol };
}
```

DART's PGS / LCP solvers both expose per-iteration residual. No new
integrator work.

### 3.3 `bullet-featherstone` implementation

Bullet's `btMultiBodyConstraintSolver` has `m_numIterationsUsed` and a
residual estimate via `m_leastSquaresResidual`. Similar wrapping:
```cpp
StepResult BulletFeatherstoneEngine::StepWithResidual(...)
{
  solver->setNumIterations(_max_iter);
  solver->setLeastSquaresResidualThreshold(_residual_tol);
  world->stepSimulation(_dt, 0, _dt);  // no internal substepping
  return { solver->m_leastSquaresResidual,
           solver->m_numIterationsUsed,
           solver->m_leastSquaresResidual <= _residual_tol };
}
```

### 3.4 `tpe` / `kinematic` — not supported

TPE and kinematic engines decline this feature (no dynamics solver, no
residual). They're only used on the kinematic tier; scenario validation
rejects an accurate-tier profile pointing at one of them.

## 4. Adaptive substepping

### 4.1 Algorithm

```cpp
bool StepAccurateTier(Engine &engine, World w, double dt,
                      const AccurateProfile &p)
{
  return StepRecursive(engine, w, dt, p, /*depth=*/0);
}

bool StepRecursive(Engine &engine, World w, double dt,
                   const AccurateProfile &p, uint32_t depth)
{
  auto result = engine.StepWithResidual(w, dt, p.residual_tol, p.max_iter);

  if (result.converged)
    return true;                                  // happy path

  if (depth >= p.max_substeps)
    return Policy::OnResidualExceeded(result, p); // gave up

  // Not converged — rewind, halve, retry twice.
  engine.RewindTo(snapshot_before_step);
  return StepRecursive(engine, w, dt / 2, p, depth + 1)
      && StepRecursive(engine, w, dt / 2, p, depth + 1);
}
```

Key facts:
- Rewind requires a pre-step snapshot. Both DART and Bullet support this
  via their world-state serialization; we use that as a checkpoint.
- Substep tree depth capped at `max_substeps` (profile-configured,
  default 16).
- Recursive structure means effective resolution is `dt / 2^depth` — from
  4 ms down to sub-microsecond.

❓ **Q34.** Rewind via world-state snapshot is expensive on large worlds
(kB–MB of state). For the accurate tier, world size is small (one arm,
one workpiece) so this is fine. But if a user puts a 100-AMR warehouse
on the accurate tier, snapshot cost could be significant. Do we (a) cap
accurate-tier world size with a warning, (b) snapshot only the accurate
engine's world (fine — they're separate under Phase 3's sidecar), (c)
optimize later? Default: (b) — Phase 3 sidecar means the accurate-tier
engine is its own world; snapshot is already bounded.

### 4.2 Failure policy

```yaml
profiles:
  arm_accurate:
    tier: accurate
    physics:
      residual_tol: 1e-6
      max_iter: 200
      max_substeps: 16
      on_residual_exceeded: error   # error | warn | event
```

Policies:
- **`error`** (default): halt the simulation, log a clear message with
  entity names and last residual norm.
- **`warn`**: log, continue. Useful in development / exploration.
- **`event`**: emit a `gz::sim::events::ResidualExceeded` on
  `EventManager`, continue. Lets user systems react (pause + alert,
  adaptive re-plan).

## 5. Profile knobs

```yaml
profiles:
  arm_accurate:
    tier: accurate
    physics:
      max_step_size: 0.0005           # baseline dt
      solver_residual_tol: 1e-6
      max_solver_iter: 200
      max_substeps: 16
      on_residual_exceeded: error
      # Optional — advanced users:
      integrator: implicit_euler      # engine-specific, informational
```

`Physics.cc` reads these via the Phase 1 `FidelityProfile` resolution; the
accurate-tier engine slot picks them up at initialization.

## 6. Validation suite

### 6.1 Reference trajectories

Two canonical tests under `test/integration/accurate/`:

**`ur10_pick_place`**:
- World: table + UR10 arm + cube on table.
- Trajectory: arm follows a scripted joint-space plan to pick and place.
- Reference: joint-space trajectory recorded from a high-accuracy
  "golden" run (DART at dt=0.1ms, residual_tol=1e-8, no substep cap).
- Test run: same scenario at profile-configured settings.
- Assertion: end-effector position error from reference ≤ 1 mm across
  5000 timesteps.

**`kuka_iiwa_manipulation`**:
- World: 7-DOF arm + contact-rich assembly task.
- Trajectory: impedance-controlled joint plan.
- Reference: similarly generated golden.
- Assertion: joint position error ≤ 0.001 rad per joint.

### 6.2 Gates

- Each validation test must pass for both dartsim and bullet-featherstone
  engines with the `arm_accurate` profile.
- CI runs these as part of the integration test suite.
- Regression detection: the golden trajectory files are versioned with
  the test; regenerating them requires a flagged CLI invocation.

❓ **Q35.** Where do golden trajectories live? Checked in (small, ~MB) or
downloaded from a fixture server at test time? Checked-in is simpler
and repo-size cost is negligible for these two tests. Default: in-repo
under `test/data/accurate/`.

## 7. Determinism within the accurate tier

Accurate-tier behavior is deterministic for a given (engine, dt, solver
tolerance, iteration cap, world state). The substepping logic is
deterministic. **Parallelism across systems** (Phase 5) can affect
order-of-evaluation; the accurate tier will document that the
`deterministic: true` runtime flag is required to guarantee bit-for-bit
reproducibility.

## 8. Interaction with Phase 3

Phase 3 delivered sidecar engines with tier ordering. Phase 4 extends the
accurate-tier slot to use `StepWithResidual` instead of bare `Step`:

```cpp
// In Physics.cc's step loop, per slot:
if (slot.tier == FidelityTier::Accurate) {
  StepAccurateTier(slot.engine, slot.world, dt, slot.profile);
} else {
  slot.engine.Step(dt);
}
```

No other cross-phase wiring changes.

## 9. Risks

**R1 — Solver residual definitions differ across engines.** DART's and
Bullet's residuals aren't directly comparable. A user migrating a profile
between engines may see different `residual_tol` values needed. Mitigation:
document per-engine tolerance guidance in the profile reference; the
validation suite's golden trajectories are per-engine.

**R2 — Substep explosion in contact-heavy scenes.** A bad contact
configuration could trigger `max_substeps` hits every step, spiking
wall-clock. Mitigation: `on_residual_exceeded: event` mode lets user
code detect and adjust (raise tolerance, switch to warn, pause).

**R3 — Golden trajectory drift.** Engine upgrades (new DART version) may
shift golden trajectories. Validation tests treat the bound as a tolerance,
not a bit-equality check — 1 mm tolerance handles normal drift. If drift
exceeds 1 mm, investigation triggers a regeneration review, not a silent
update.

**R4 — Implementation gap in `bullet-featherstone`.** If Bullet's residual
exposure is weaker than assumed (we need to verify `m_leastSquaresResidual`
semantics), we may need a wrapping residual computation. Flagged for
Phase 4 kickoff; prototype first week.

❓ **Q36.** For the `on_residual_exceeded` event policy, do user systems
need **per-entity** identification (which body had the highest residual)
or just the global residual? Per-entity is more actionable; requires
walking the constraint Jacobian which may not be cheap. Default: global
residual + body count, per-entity on explicit request in the profile.

## 10. Testing

- **Unit**: `StepWithResidual_TEST` — single-body and constraint-heavy
  worlds, assert returned residual/iteration values make sense.
- **Unit**: `AdaptiveSubstep_TEST` — construct a world that won't
  converge at any dt, verify `max_substeps` honored, policy dispatch
  correct.
- **Integration**: validation suite as described in §6.
- **Benchmarks**: not a throughput focus, but record wall-clock-per-sim-s
  on the two reference scenes across engines. Regression alarm at +20%.

## 11. Milestones

| Week | Deliverable |
|---|---|
| 1 | `StepWithResidual` feature declared; dartsim prototype; unit test. |
| 2 | bullet-featherstone implementation; dartsim polish. |
| 3 | Adaptive substepping in Physics.cc; policy dispatch. |
| 4 | Validation suite infrastructure; golden trajectory generator CLI. |
| 5 | Reference trajectories recorded + checked in. |
| 6 | Validation tests wired in CI; gate enforced. |

Six weeks. Parallelizable with Phase 5; most likely slip is
bullet-featherstone residual plumbing (week 2) if Bullet's API is less
cooperative than assumed.

## 12. Exit criteria

- `StepWithResidual` feature in `gz-physics` main interface.
- Implementations in dartsim and bullet-featherstone.
- `Physics.cc` adaptive substepping for accurate tier.
- Validation tests green.
- `ur10_cell_accurate.yaml` scenario available and running under CI.
