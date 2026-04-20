# Phase 3 — Sidecar Engines

Status: Draft
Owner: Nathan Koenig
Scope: Phase 3 of the Adjustable-Fidelity initiative
Depends on: [Phase 2](phase-2-kinematic-tier.md) (kinematic engine exists)
Blocks: Phase 4 (accurate-tier substepping expects its own engine instance)

## 1. Context

Phase 2 made the kinematic engine real but restricted each world to a
single tier. Phase 3 removes that restriction: a world can host
`kinematic`, `approximate`, and `accurate` entities simultaneously, each
routed to the physics engine appropriate for its tier. This is the
"warehouse with 100 AMRs **and** a UR10 manipulation cell" case.

Coupling between tiers is **one-way static** (locked in design):
lower-tier entities appear to higher-tier engines as immovable fixtures,
updated each step. Higher-tier entities do not push lower-tier ones.

## 2. Goals / Non-goals

Goals:
- `Physics.cc` holds a map `FidelityTier → unique_ptr<Engine>`.
- Entities routed to exactly one engine by their `FidelityTier` component.
- Deterministic per-step execution order:
  `kinematic → approximate → accurate`.
- Cross-tier pose synchronization: after each engine steps, its entities'
  poses are written back to the ECM; higher-tier engines read these as
  static fixture updates in their world before their own step.
- Scenario loader validates: engines declared in `scenario.physics.engines`
  are available, tiers used by entities have an engine declared.
- `mixed_20_robot` scenario: AMRs (kinematic) + arm (approximate) at
  ≥ 1× RTF.

Non-goals:
- Two-way coupling (force exchange across tier boundaries).
- More than three tiers (the three are enum-enumerated; extending means
  a design revision).
- Per-link tier overrides within a single model — still model-scoped.
- Distributed execution across machines — Phase 7.

## 3. Architecture

### 3.1 Data model in `Physics.cc`

```cpp
class Physics::Impl {
  struct EngineSlot {
    FidelityTier tier;
    std::string  engine_plugin;        // e.g. "gz-physics-kinematic"
    std::unique_ptr<EngineInterface> engine;
    std::unordered_map<Entity, PhysicsEntityHandle> local_handles;
  };
  // Ordered: kinematic first, accurate last:
  std::vector<EngineSlot> engines_;

  // Entity → which slot owns it:
  std::unordered_map<Entity, size_t> entity_slot_;
};
```

Each slot owns an independent `gz::physics` engine instance. The map is
constructed at scenario load from `scenario.physics.engines`.

### 3.2 Per-step sequence

```
Physics::Update(_dt):
  # 1. Intake: any ECM mutations from PreUpdate update engine state
  for slot in engines_:
    sync_ecm_to_engine(slot)   # pose cmds, new/removed entities

  # 2. Step in tier order (kinematic -> approximate -> accurate)
  for slot in engines_ (ordered):
    slot.engine.Step(_dt)

    # 3. Writeback: final entity poses in this tier go to ECM
    sync_engine_to_ecm(slot)

    # 4. Propagate as static fixtures to higher tiers
    for higher_slot in engines_ after slot:
      for entity in slot.entities:
        higher_slot.engine.SetStaticFixture(entity, final_pose)
```

Key point: step #4 runs before the higher-tier Step. By the time the
accurate-tier engine steps, all lower-tier entities have been updated as
static fixtures in its world. One-way coupling falls out naturally.

### 3.3 Static-fixture projection

When a kinematic AMR appears in the accurate tier's engine world, it's
instantiated with:
- Same collision shapes as the original.
- `is_static = true`.
- Pose updated every step from ECM.
- **No** mass, **no** joints, **no** ownership of dynamic properties.

Performance: each higher-tier engine maintains a stable set of proxy
bodies (one per lower-tier entity). Pose updates are O(n) memory writes
per step, no engine-internal recomputation beyond what a static fixture
move triggers (typically a broadphase reinsert).

❓ **Q32.** Does the higher-tier engine need **any** collision with
lower-tier proxies, or do we skip proxy creation entirely for performance?
The "proxy" only matters if, for example, the UR10 arm might swing into
an AMR's path. In most realistic scenarios, manipulation cells and AMR
travel lanes are spatially separated. Default: create proxies (safe
behavior, small cost). Scenario can opt out with a
`physics.cross_tier_collision: false` flag per tier pair.

### 3.4 Engine capability negotiation

Each engine declares its supported features via `gz::physics` feature
tags. `Physics.cc` at startup:
- Confirms each declared engine supports the feature set its tier needs.
- For kinematic: minimal subset (see [Phase 2 §4.1](phase-2-kinematic-tier.md)).
- For approximate: standard DART/TPE feature set.
- For accurate: approximate's feature set + `StepWithResidual` (Phase 4).

If a scenario declares `accurate: dartsim` but dartsim hasn't shipped
`StepWithResidual` yet (Phase 4 precondition), scenario load fails with a
clear error.

## 4. Scenario integration

Scenario schema (v1) already expresses this:
```yaml
physics:
  engines:
    kinematic:   gz-physics-kinematic
    approximate: dartsim
    accurate:    bullet-featherstone
```

Phase 3's scenario loader:
1. Reads the engines block.
2. Instantiates one engine per declared tier.
3. Validates features.
4. Builds the `engines_` vector in canonical order.

If a tier is used by at least one matched override but no engine is
declared, scenario load fails.

## 5. Cross-tier pose sync detail

Lower-tier entity poses are synced into higher-tier engines once per step
via a new feature interface `SetStaticFixturePose`:

```cpp
namespace gz::physics {
  struct SetStaticFixturePose : public virtual FeaturePolicy3d {
    // Set the world-space pose of a static body without triggering
    // force/torque recomputation. Used for cross-tier proxies.
    virtual void SetStaticFixturePose(
        const Identity &_body,
        const math::Pose3d &_pose) = 0;
  };
}
```

Implementations:
- DART: `BodyNode::setTransform(pose); skeleton->computeForwardKinematics()`
  skipped (body is marked kinematic-only).
- Bullet-featherstone: `btCollisionObject::setWorldTransform(pose);
  dynamicsWorld->updateSingleAabb()`.
- Kinematic engine: no-op (kinematic is always the source, never a
  receiver of cross-tier proxies).

Feature is added to both dartsim and bullet-featherstone as part of
Phase 3.

## 6. Memory layout in ECM

Each entity carries a single `components::FidelityTier` component
(Phase 1 delivered). Phase 3 adds a lightweight routing cache:

```cpp
// Internal to Physics.cc; not user-visible.
components::PhysicsSlot { size_t slot_index; };
```

Attached on first tier resolution; read per step to route without a
`FidelityTier → slot_index` map lookup. Maintained when tier changes
(future dynamic-fidelity work).

## 7. Mixed-world scenario

Reference scenario `examples/scenarios/mixed_20_robot.yaml`:
- Warehouse world SDF.
- 20 AMRs (kinematic).
- 2 robot arm cells (approximate).
- 1 bounded-residual arm cell (accurate) — included even though it will
  slow the sim, to exercise the three-tier path.

Gate: RTF ≥ 1× on reference box. Single-tier `warehouse_100_amr` still
≥ 3× (no regression from Phase 2).

## 8. Risks

**R1 — Engine startup cost.** Three engines = three initializations. Adds
0.5–2 s to scenario load. Acceptable; flagged in release notes.

**R2 — Duplicate broadphase over static world geometry.** Each engine has
its own copy of wall/shelving collision data. Memory overhead ~3× baseline.
At warehouse scale this is tens of MB — not a blocker.

**R3 — Proxy pose sync latency.** If the kinematic engine steps at the
world timestep (4 ms) and the accurate engine at a substep (0.5 ms,
Phase 4), proxies are only refreshed each *accurate* step when its
parent kinematic step completes. A fast-moving AMR near an arm might
present a stale proxy for up to 4 ms. Acceptable for the scenarios we
target (arms work on stationary payloads); documented.

**R4 — Scenario load validation gaps.** Users might declare engines
that aren't installed. Fails gracefully with a message; track this in the
test suite.

❓ **Q33.** If a scenario uses only one tier (say all kinematic), do we
instantiate unused engines (for the declared-but-unused tiers in the
engines map) or skip them? Skipping is faster startup; instantiating
matches the spec more literally. Default: skip — instantiate on demand
when the first entity routes to that tier.

## 9. Testing

### 9.1 Unit

- `EngineRouter_TEST` — scripted entity additions, verify each lands in
  the correct engine.
- `StaticFixtureSync_TEST` — move a kinematic entity, assert the pose
  appears in the accurate engine's static set.
- `ScenarioPhysicsValidation_TEST` — scenarios with missing engine
  declarations fail cleanly.

### 9.2 Integration

- `mixed_20_robot.yaml` runs; per-tier entity counts asserted after load;
  1000-step cycle completes; RTF recorded.
- Contact sensor test across tiers: kinematic AMR bumps a DART-simulated
  cart (with the cart marked as a static fixture to AMR side, dynamics
  on its own side). Expected: contact fires on AMR side, no force
  transferred.

### 9.3 Benchmarks

- `mixed_20_robot_bench.cc` — 60 s, record RTF per tier and aggregate.
- Memory profile — confirm < 3× vs single-tier baseline.

## 10. Milestones

| Week | Deliverable |
|---|---|
| 1 | `Physics.cc` refactor: extract single-engine path into an EngineSlot; flag-gated dual-engine path. |
| 2 | Multiple engines instantiated from scenario; routing by tier; no cross-tier sync yet. |
| 3 | `SetStaticFixturePose` feature: add to `gz-physics` interface + dartsim + bullet-featherstone. |
| 4 | Cross-tier sync loop; proxy lifecycle (create/update/remove). |
| 5 | `mixed_20_robot.yaml` integration; end-to-end pass. |
| 6 | Benchmark + memory profile; tuning. |
| 7 | Phase 3 tagged. |

Seven weeks. Phase 4 starts when `StepWithResidual` is needed for a
real scenario (can begin in parallel after week 3 lands).

## 11. Exit criteria

- Multi-engine `Physics.cc` landed.
- `SetStaticFixturePose` feature upstreamed through `gz-physics`.
- `mixed_20_robot.yaml` passes integration + benchmark gate.
- Single-tier performance unchanged from Phase 2.
