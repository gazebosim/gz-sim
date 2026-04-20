# Phase 6 — Auto-Leveling for Large Worlds

Status: Draft
Owner: Nathan Koenig
Scope: Phase 6 of the Adjustable-Fidelity initiative
Depends on: [Phase 5](phase-5-parallel-systems.md); integrates with the
existing LevelManager + scenario generator
Blocks: Phase 7 (distributed), which expects meaningful per-level
partitioning to distribute.

## 1. Context

The existing Levels/Performers system
([LevelManager.hh](../../include/gz/sim/LevelManager.hh)) partitions a
world into manually-authored spatial zones. For a 10,000 m² warehouse
with 200 AMRs, hand-authoring levels is a non-starter.

Phase 6 introduces **auto-leveling**: the scenario declares a
leveling strategy; the runtime computes levels from the world geometry
and generator output. No per-level YAML required.

Secondary goal: make sure levels **actually buy throughput** at scale
— auto-gen is pointless if the current LevelManager can't sustain 200
performers without melting down.

## 2. Goals / Non-goals

Goals:
- Scenario-level strategy declaration (grid, octree, or manual).
- Runtime level construction at scenario load, after entity spawn.
- Performer tracking already works — integrate without rewrite.
- Level load/unload batched through the archetype ECM's command buffer
  (efficient under 0a).
- Stress target: 200 AMRs in 10,000 m² warehouse, simulation stable for
  1 hour of sim time, RTF ≥ 3× (not 5× — levels-loading cost comes out
  of the parallelism budget).

Non-goals:
- Dynamic level re-partitioning (a level is defined once; performers
  move in/out but levels don't redraw themselves).
- Cross-level entity migration (entities belong to exactly one level).
- Semantic leveling (grouping by model type instead of space) —
  deferred indefinitely.

## 3. Scenario declaration

Add to Phase 1's schema:
```yaml
world:
  sdf: worlds/warehouse.sdf
  levels:
    strategy: grid              # grid | octree | manual | none
    # For grid:
    cell_size: [20.0, 20.0, 999.0]    # m, xy-grid; z is huge = flat
    origin:    [0.0, 0.0, 0.0]
    # For octree:
    max_depth:     4
    min_cell_size: [5.0, 5.0, 5.0]
    # Performer buffer around each performer in meters:
    buffer_distance: 10.0
    # Entities always loaded regardless of performer proximity:
    always_loaded:
      - name: "warehouse_structure"
      - tag: "static_infrastructure"
```

Validation at scenario load:
- For `grid`, `cell_size` values all > 0.
- For `octree`, `min_cell_size` * 2^`max_depth` > world extents.
- `buffer_distance` < min(`cell_size`) to avoid a performer spanning
  many levels simultaneously.

## 4. Leveling strategies

### 4.1 Grid

Simplest and likely sufficient for warehouses. World's XY extents
computed from loaded entities' AABBs; XY divided into cells of
`cell_size`. Each cell is a Level. Entities are assigned to the cell
containing their centroid.

Pros: O(1) performer-to-level lookup via division. Trivially parallel
assignment at load.
Cons: Uneven distribution if the world is irregular (buildings with
narrow corridors).

### 4.2 Octree

More general. Recursively subdivide the world AABB; stop when a cell
contains few enough entities (< `target_entities_per_level`, default
50) or reaches `min_cell_size` / `max_depth`.

Pros: Adaptive; handles irregular entity distribution.
Cons: Point-location is `O(log depth)` instead of `O(1)`. More memory.

Use when grid doesn't give balanced distribution.

### 4.3 Manual

Existing behavior — levels in the SDF via the legacy `<level>` tags.
Scenario declares `strategy: manual` and LevelManager loads as before.

### 4.4 None

No leveling. Entire world always loaded. Fine for small worlds.

## 5. Load flow

```
1. Scenario loads; SDF world entities created in ECM.
2. scene.generators run → bulk entity additions.
3. If scenario.world.levels.strategy != "none":
   3a. Collect AABBs of all entities.
   3b. Run strategy → produce LevelSpec list.
   3c. For each LevelSpec, create a Level in LevelManager; register
       member entities; initially unload unless a performer is near.
4. Phase 1's plugin resolution proceeds; plugin instantiation respects
   level state (entities in unloaded levels have their plugins deferred
   until level load).
5. Simulation starts; performer tracking drives load/unload.
```

### 5.1 Performer identification

Performers are entities with a `components::Performer` tag — today set
via SDF or programmatically. In Phase 6, a scenario `overrides` rule can
set it:
```yaml
overrides:
  - match: { model: "amr_*" }
    performer: true
```

So all 100 AMRs in the warehouse scenario become performers automatically.

## 6. Efficient load / unload under archetypes

Under the archetype ECM (0a), level load is already faster than legacy
(batch through one command buffer, one Commit). Phase 6 leverages this
further:

- Entity creation for level load is queued as a batch of `CreateEntity`
  + `AddComponent(...)` commands.
- One `Commit()` applies the batch.
- Archetype-graph hit rate is high: entities in the same level often
  share archetypes (same SDF template), so most land in the same
  destination chunks.

Bench gate: loading/unloading a 20m × 20m cell containing ~50 entities
should complete in < 5 ms. If it doesn't, level transition jitter
becomes visible to systems.

## 7. Integration with generator

Phase 1's scene generators (grid, random, explicit) produce entities
whose poses are known at scenario-parse time. Phase 6 piggybacks:

```yaml
scene:
  generators:
    - template: model://amr_mir250
      count: 200
      name_prefix: amr_
      layout: { type: grid, origin: [0,0,0], spacing: [5,5,0], rows: 20 }
```

After generator runs, the leveling pass sees 200 entities and produces
levels. Generator entities are tagged `components::Performer` if the
scenario's `overrides` say so; performers are always kept loaded (the
leveling pass skips them when deciding initial unload state).

## 8. Memory behavior

Critical: the sim must fit in memory.

For a 10,000 m² warehouse with:
- 200 AMRs @ ~100 KB ECM state each = 20 MB.
- ~50,000 static collision primitives = ~5 MB (with FCL BVH).
- Kinematic engine broadphase structure = ~20 MB.
- Sensor state (sparse, most AMRs kinematic tier so no rendered sensors)
  = < 10 MB.

Total < 100 MB. Well within a dev laptop. Levels help most by keeping
only nearby statics loaded in sensor-rendering tiers; kinematic tier
needs global collision info regardless.

## 9. Constraints on the kinematic engine

The kinematic engine (Phase 2) is global — it tracks every kinematic
entity regardless of level. This is intentional: AMRs shouldn't tunnel
through each other just because they happen to be in adjacent levels.

Trade-off: the kinematic engine's broadphase holds the full world. At
our scale this is fine (FCL's BVH is sparse-friendly). If we ever scale
past 1000 AMRs, revisit.

❓ **Q40.** For accurate-tier entities, should we require them to live in
a single level (no cross-level arm), or allow cross-level spanning?
Arms are small; a scenario would usually put them entirely in one cell.
Disallowing cross-level simplifies the engine. Default: disallow,
validate at scenario load.

## 10. Testing

- **Unit**: grid strategy produces expected cell layout for hand-authored
  world extents. Octree strategy balance-tests on synthetic entity
  distributions.
- **Integration**: `warehouse_200_amr.yaml` scenario loads, runs for 1
  sim-hour, memory stable, no leaks, RTF ≥ 3.
- **Performer motion**: a scripted AMR trajectory crosses level
  boundaries repeatedly; observed load/unload counts match expectations;
  no dropped contact events.
- **Benchmarks**: time-to-first-step on a 200-AMR scene < 5 s.

## 11. Risks

**R1 — Level transition jitter.** Loading a level on the fly costs ms;
if it lands on the critical path of the main loop, step duration spikes.
Mitigation: preemptive loading based on performer velocity (load the
next level before the performer arrives). Configurable lookahead.

**R2 — Edge-case performers at cell boundaries oscillate.** A performer
exactly at the cell edge may trigger load/unload every step. Mitigation:
hysteresis — load at buffer_distance, unload at buffer_distance * 1.5.

**R3 — Memory overhead of always-loaded infrastructure.** Users tag more
as `always_loaded` than necessary. Scenario linting warns if
`always_loaded` > 10% of static entities.

**R4 — Interaction with sidecar engines.** Each tier engine has its own
world; level load/unload must sync to all of them. Phase 3's sync is
general enough but the volume of sync traffic at level-transition time
was not benchmarked. Measure early in Phase 6.

❓ **Q41.** Do you want level boundaries **visible** in the GUI (lines on
the ground, debug viz), especially during scenario development? Cheap to
implement via a debug overlay system; genuinely helpful. Default: yes,
gated behind `runtime.debug.show_levels: true`.

## 12. Milestones

| Week | Deliverable |
|---|---|
| 1 | Scenario schema extension for levels; validation. |
| 2 | Grid strategy landed + tests. |
| 3 | Octree strategy + balance tests. |
| 4 | Integration with LevelManager's load/unload; batch through archetype command buffer. |
| 5 | Performer auto-tagging; hysteresis; preemptive load-ahead. |
| 6 | `warehouse_200_amr.yaml` scenario; run 1 sim-hour; meet gates. |

Six weeks.

## 13. Exit criteria

- Grid and octree strategies shipped.
- `warehouse_200_amr.yaml` scenario runs for 1 sim-hour at ≥ 3× RTF.
- Level transitions < 5 ms.
- Memory stable (no leak over 1 sim-hour).
- Debug visualization of level boundaries available.

## 14. What unblocks

Phase 7 (distributed) — now has meaningful partitioning to distribute
across machines.
