# Phase 2 — Kinematic Tier (Engine + Motion System + Nav2 Compat)

Status: Draft
Owner: Nathan Koenig
Scope: Phase 2 of the Adjustable-Fidelity initiative
Depends on: [Phase 1](phase-1-scenarios.md) (for `FidelityTier` and plugin
wiring); 0a–0e (ECM foundation)
Blocks: Phase 3 (sidecar engines relies on a real kinematic engine existing)

## 1. Context

Phase 2 is the first phase that moves the RTF needle. Three deliverables:

1. **`gz-physics-kinematic`** — a new `gz::physics` engine plugin, FCL-backed,
   that integrates poses from velocity commands and resolves inter-entity
   overlaps without doing dynamics.
2. **`kinematic_motion` system** — replaces `diff_drive` et al. for
   kinematic-tier entities. Writes pose and joint-state directly to the
   ECM.
3. **Nav2-compatibility wiring** — `cmd_vel` in; `odom`, `tf`,
   `joint_states`, contact events out, identical topics and schemas to
   today's `diff_drive`.

Target gate: `examples/scenarios/warehouse_100_amr_fast.yaml` achieves
**≥ 3× RTF** on a reference workstation. Nav2 drives an AMR round-trip
(goal → path → arrival) end-to-end without code changes on the Nav2 side.

The final 5× RTF target is a Phase 5 gate (needs parallel PreUpdate/Update);
Phase 2 proves the kinematic path works at scale.

## 2. Goals / Non-goals

Goals:
- New gz-physics engine plugin implementing a minimal feature set:
  `ForwardStep`, `GetContactsFromLastStep`, `SetWorldPoseCommand`,
  `ConstructEmptyWorld`, entity hierarchy management.
- `kinematic_motion` system supporting differential, Ackermann, and
  mecanum motion models (covered by scenario params).
- Contact event emission compatible with the existing contact sensor.
- `Physics.cc` gates entities by `FidelityTier` and routes kinematic
  entities exclusively to the new engine.
- Nav2 integration validated end-to-end.

Non-goals:
- Sidecar coexistence with DART/Bullet in the same world — that's Phase 3.
  For Phase 2 a world is either all-kinematic or all-DART (legacy), not
  both at once.
- Full dynamics / friction / restitution. By design.
- Pushable objects. Kinematic entities do not transfer momentum.
- Multi-machine distribution — Phase 7+.

## 3. Architecture

```
   scenario YAML              SDF world
        │                          │
        └─────────── loader ───────┘
                        │
                        ▼
                     ECM (archetype)
                        │
     ┌──────────────────┼───────────────────────────────────────┐
     ▼                  ▼                                       ▼
 kinematic_motion   (other systems...)                   Physics.cc
 (PreUpdate)                                            (Update/PostUpdate)
     │                                                         │
     │  writes pose + joint state                              │
     │  publishes odom/tf                                      ▼
     └────────────────► ecm ◄──── gz-physics-kinematic ────────┘
                                    (FCL broadphase +
                                     narrowphase +
                                     push-out resolver)
```

The motion system is responsible for turning commands into **desired**
poses/joint-states. The kinematic engine is responsible for collision
resolution and contact generation. Clean separation.

## 4. `gz-physics-kinematic` engine

### 4.1 Feature set implemented

Minimal subset of `gz::physics` features:

| Feature | Used for |
|---|---|
| `ConstructEmptyWorld` | world creation |
| `AddWorldModel`, `AddModelLink` | entity hierarchy registration |
| `ForwardStep` | main step — resolves overlaps, updates contacts |
| `GetContactsFromLastStep` | contact sensor data |
| `SetWorldPoseCommand` | motion system writes target poses |
| `GetShapeBoundingBox`, `AttachBoxShape`, `AttachSphereShape`, `AttachMeshShape` | collision geometry |
| `RemoveModel`, `RemoveLink` | cleanup |

Features intentionally **not** implemented:
- `FreeGroup` / joints — kinematic entities have no joint constraints.
  Joint state is set by the motion system directly as a ledger.
- `ExternalForce`, `WorldWrench`, `JointForce` — no dynamics.
- `SetJointVelocity` as a dynamics input — motion system bypasses.

If a feature isn't implemented and a caller requests it, the standard
`gz::physics` feature-negotiation mechanism handles the refusal. `Physics.cc`
checks capabilities during setup and warns loudly on a requested feature
the kinematic engine doesn't support.

### 4.2 Step algorithm

```
ForwardStep(_dt):
  1. for each entity E registered in world:
       desired_pose := ecm-written SetWorldPoseCommand target, or identity
       proxy[E].pose := desired_pose
  2. broadphase: FCL BVH update, produce overlap pair list
  3. for each overlap pair (A, B):
       narrowphase via FCL penetration depth
       if penetrating:
         push_out_vector := resolve_pushout(A, B)
         if A.is_static: apply to B
         elif B.is_static: apply to A
         else: split (each takes half)
  4. (optional) iterate pushout up to N=4 for stability (default N=2)
  5. write final poses back to state
  6. generate ContactPoint records for contact sensors
```

Pushout on Step 3 is the entire "physics" — no mass, no friction, no
restitution. An AMR that drives into a wall stops. Two AMRs that drive
into each other each get pushed back half. The velocity in the motion
system is unchanged — next step, if cmd_vel is still commanding into the
wall, it re-penetrates and re-pushes out, producing a stable "stuck"
state. Contact events fire each step in this configuration, which is
exactly what Nav2 safety layers expect.

### 4.3 FCL integration

FCL shapes constructed from the entity's `<collision>` elements:
- `<box>` → `fcl::Box`
- `<cylinder>` → `fcl::Cylinder` (or 2D hull approximation, see §4.4)
- `<sphere>` → `fcl::Sphere`
- `<mesh>` → `fcl::BVHModel<fcl::OBBRSS>` built from mesh vertices/indices

AMRs typically have `<cylinder>` or `<box>` footprints. The plugin
prefers the cheapest narrowphase that fits the geometry — sphere-vs-sphere
is ~10ns per pair.

Broadphase: FCL's `DynamicAABBTreeCollisionManager`. Incremental update
on pose writes; full rebuild every N steps (default 1000) to avoid tree
imbalance for long-running sims.

### 4.4 2D hull for mobile bases

Entities with a `kinematic.hull: 2d` flag (set via profile option) are
collapsed to their XY projection for collision purposes. Implementation:
build an `fcl::Convex` from the 2D convex hull of collision vertices at
the base's ground-plane pose. Used for AMRs on flat floors — ~5× faster
than 3D mesh narrowphase, and the full 3D accuracy is noise when an AMR
never leaves Z=floor.

Non-AMR entities (lifts, forklifts, overhead cranes) stay 3D via the
default path.

❓ **Q28.** Do you want `2d` hull opt-in (user adds the flag) or auto-detect
(plugin decides based on "entity never moves in Z")? Auto-detect is
fragile; opt-in is more boring but predictable. My vote: opt-in, defaulted
in the `kinematic` profile template.

### 4.5 Static vs. dynamic

Static entities (walls, shelving) register once with `is_static=true`.
They stay in the broadphase and participate in overlap checks but their
pose never updates from the motion system. Dynamic entities (AMRs) pose
updates per step.

Performance: 100 AMRs × ~10k static collision primitives in a warehouse
is the realistic shape. FCL's DynamicAABBTree is sparse-friendly and
handles this comfortably.

### 4.6 Contact data schema

Contact records from the engine populate `components::ContactSensorData`
exactly as DART/Bullet do. Contact sensors attached to kinematic entities
receive their expected `msgs::Contacts` payload.

Contacts expose:
- `collision1`, `collision2` entity references
- `position` — world-frame contact point (midpoint of overlap)
- `normal` — overlap resolution direction
- `depth` — penetration depth (positive = interpenetrating)
- `wrench` — **zero** (no forces in kinematic mode; documented; Nav2
  doesn't consume this)

## 5. `kinematic_motion` system

### 5.1 Responsibilities

- Subscribe to cmd_vel topic(s) per model.
- On PreUpdate: integrate cmd_vel → desired pose delta → write target pose
  via `SetWorldPoseCommand` on the kinematic engine (routed through the
  ECM, not called directly).
- Update joint positions on wheels (for `joint_states` continuity).
- Publish odometry, TF.
- Emit a `halt` signal if the kinematic engine reports a contact that
  blocks forward motion (configurable — stop, slow, or ignore).

### 5.2 Supported motion models

The scenario profile selects:
```yaml
kinematic_drive:
  type: differential      # differential | ackermann | mecanum
  wheel_radius: 0.1
  track: 0.4
  wheelbase: 0.5          # ackermann only
  max_linear_accel: 2.0   # optional, for smoothing spikes
  max_angular_accel: 3.0
```

Per-model overrides go in the scenario's plugin params. The system reads
these at `ConfigureFromYaml`.

### 5.3 Integration step

```
PreUpdate:
  for each kinematic-tier model:
    twist := latest cmd_vel (smoothed)
    dt    := step size
    pose  := ecm.Component<Pose>(model).value
    pose.pos += rotate(twist.linear * dt, pose.yaw)
    pose.yaw += twist.angular.z * dt
    ecm.SetComponentData<KinematicTargetPose>(model, pose)
    jpos_left  += (twist.linear - twist.angular.z * halfTrack) * dt / wheelRadius
    jpos_right += (twist.linear + twist.angular.z * halfTrack) * dt / wheelRadius
    ecm.SetComponentData<JointPosition>(leftWheel, jpos_left)
    ecm.SetComponentData<JointPosition>(rightWheel, jpos_right)
```

After PreUpdate commits, Physics.cc hands off to the kinematic engine,
which may push poses back due to collision — the motion system reads the
**post-pushout** pose next step to compute the next delta, so a wall-bumped
AMR doesn't drift through the wall over time.

### 5.4 Odometry & TF

Implemented by delegating to the existing `odometry_publisher` and
`pose_publisher` systems — same plugins `diff_drive` ecosystems already use.
No new pose publishing logic.

### 5.5 Joint state continuity

Nav2's TF tree includes wheel joints. `joint_state_publisher` system reads
`components::JointPosition` and publishes. Since kinematic_motion writes
those components each step, downstream sees a continuous joint-state
stream identical to dynamics-driven output.

## 6. `Physics.cc` tier gating

Today [Physics.cc](../../src/systems/physics/Physics.cc) sees every entity
with a `components::CollisionElement`. Phase 2 filters by tier:

```cpp
void Physics::Update(...) {
  for each entity with collision:
    tier := ecm.Component<FidelityTier>(model_of(entity))
    if tier == Kinematic:
      register/update in kinematic_engine
    else:  // Approximate, Accurate
      register/update in legacy_engine (dart/bullet)
  step both engines (phase 2: exactly one is active per world; phase 3
    is when both coexist)
}
```

In Phase 2 we restrict scenarios to **one tier per world** at load time.
A world cannot host both kinematic and non-kinematic models. This is a
temporary simplification — Phase 3 relaxes it.

Scenario loader validation: if `overrides` declare multiple tiers, refuse
to load with a clear error pointing at Phase 3's readiness.

## 7. Nav2 integration validation

End-to-end test harness (`test/integration/nav2_kinematic/`):
- Spin up a small Nav2 stack (via `nav2_bringup`) against a simulated AMR.
- Command a `NavigateToPose` goal.
- Assert:
  - cmd_vel received by sim matches TF-derived robot motion.
  - odom drift within tolerance.
  - Contact sensor fires when robot hits a simulated obstacle placed in
    the path.
  - Arrival at goal within time budget.

Runs in CI as an opt-in heavy test (needs Nav2 install).

❓ **Q29.** CI resources — the Nav2 round-trip test needs a ROS 2 env.
Run it in a Docker stage on every PR, or gate behind a nightly?
Nightly is cheaper; PR-time catches regressions sooner. Default: nightly
+ manual trigger, escalate to per-PR if we see drift.

## 8. Benchmarks

`bench/warehouse_100_amr.cc`:
- Warehouse SDF with 10k static collision primitives.
- 100 AMRs spawned via scenario generator.
- Each AMR commanded a pseudo-random cmd_vel trajectory.
- 60 s simulated time.
- **Gate**: RTF ≥ 3.0× on the reference box (32-core, 64GB).
- Soft goal: same box, RTF ≥ 5.0× with planned Phase 5 parallelization.

`bench/nav2_loop_rtf.cc`:
- Same scene + Nav2 stack running per AMR.
- **Gate**: RTF ≥ 1.5× (Nav2 overhead matters).

## 9. Risks

**R1 — FCL narrowphase cost at warehouse scale.** 100 AMRs ×
~50 broadphase overlaps each ≈ 5000 narrowphase checks/step. Measured on
FCL literature: < 500 μs. Should be fine; benchmark early.

**R2 — Pushout instability.** Multi-body overlaps (3+ bodies mutually
overlapping) can oscillate. Mitigation: iteration cap, and declare
"simulated failure" when iteration cap hits (emit event, log). Users
should not design scenarios that routinely trigger this.

**R3 — Step-size coupling.** Motion system integrates at step size. Too
large → AMR tunnels through thin walls. Kinematic engine has no CCD.
Mitigation: warn at scenario-load if step_size * max_speed > min_wall_thickness
heuristic.

**R4 — Nav2 compatibility drift.** Nav2 evolves; topic names and QoS
sometimes shift. The integration test pins a Nav2 version; upgrades are
tracked as a separate item.

❓ **Q30.** `halt` behavior on blocked motion: (a) stop the AMR until
cmd_vel reverses, (b) slow asymptotically as obstacle approaches, (c)
nothing — let Nav2's safety layer handle it via contact events. Default:
(c) — we're a simulator, not a controller. Contact events give Nav2 what
it needs.

❓ **Q31.** Do you want a `force_free_space: true` option that lets
users opt out of collision entirely on the kinematic engine for pure
flow analyses (no walls, no collisions, maximum throughput)? This is a
~10% more speed on the 100-AMR test. Simple to add.

## 10. Milestones

| Week | Deliverable |
|---|---|
| 1 | Skeleton `gz-physics-kinematic` plugin; world + shape construction; ForwardStep no-op. |
| 2 | FCL broadphase + narrowphase integration; overlap detection working in tests. |
| 3 | Pushout resolver; stability tuning; unit tests for collision cases. |
| 4 | `kinematic_motion` system: differential + mecanum + ackermann; integrated in Physics.cc flow. |
| 5 | Contact event schema; contact sensor integration test green. |
| 6 | Warehouse scene bench; hit 3× RTF gate. |
| 7 | Nav2 end-to-end test green. |
| 8 | Documentation, examples, Phase 2 tagged. |

Eight weeks. Two-week slip acceptable. Most likely slip: FCL narrowphase
performance tuning (week 3), depending on what real warehouse geometry
looks like.

## 11. Exit criteria

- `gz-physics-kinematic` shipped as a plugin under `gz-physics/`.
- `kinematic_motion` system shipped under `gz-sim/src/systems/`.
- `warehouse_100_amr_fast.yaml` benchmark ≥ 3× RTF.
- Nav2 round-trip test passes.
- Phase 3 unblocked (sidecar engine hooks in Physics.cc exist but gated).
