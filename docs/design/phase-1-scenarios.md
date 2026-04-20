# Phase 1 — Scenarios

Status: Draft
Owner: Nathan Koenig
Scope: Phase 1 of the Adjustable-Fidelity initiative
Depends on: [0a](phase-0a-archetype-ecm.md), [0b](phase-0b-ecm-integration.md); 0d (YAML Configure) can land in parallel
Blocks: Phase 2 (kinematic engine + system), 3 (sidecar engines), 4 (accurate tier), 5+

## 1. Context

Phase 1 introduces **Scenarios** — YAML files that fully describe a runnable
Gazebo simulation. A scenario references an SDF world and owns everything
simulator-specific: fidelity profiles, plugin wiring, procedural spawning,
initial state, physics settings, runtime knobs.

Rationale (from design discussion):
- SDF stays simulator-agnostic — Gazebo-specific behavior leaves it.
- One SDF can be exercised as a 100-AMR kinematic workflow sim, a 20-robot
  medium-fidelity scene, or a slow bounded-residual accuracy test — by
  swapping the scenario, not the SDF.
- Long-term, `<plugin>` tags migrate out of SDF into scenarios entirely.

Phase 1 ships: YAML schema v1, loader/resolver, `FidelityTier` component,
CLI entry points (`gz sim scenario.yaml`, `gz sim scenario resolve`,
`gz sim scenario convert`), and the resolved-report emitter. It does **not**
ship the kinematic engine, sidecar engines, or any fidelity behavior
change — those begin in Phase 2. Phase 1's job is plumbing.

## 2. Goals / Non-goals

Goals:
- Schema v1 frozen (this doc § 4).
- `gz-sim-scenario` library: parse, validate, compose (`extends`), resolve
  rules against loaded entities, emit `FidelityTier` and plugin lists.
- Sub-commands: `gz sim <scenario.yaml>`, `gz sim scenario resolve`,
  `gz sim scenario convert`.
- Deterministic resolved-report output for CI golden-file testing.
- Legacy path — `gz sim world.sdf` without scenario — continues to work
  unchanged (implicit default scenario, approximate tier, SDF plugins).

Non-goals:
- The actual `kinematic` or `approximate` or `accurate` engines behaving
  differently. Phase 1 only **labels** entities; consumption is Phase 2+.
- ROS 2 bridge config (`bridges:` section reserved but not implemented).
- Logging / recording config (`logging:` reserved but not implemented).
- Runtime re-loading of a scenario (deferred indefinitely).
- Dynamic fidelity switching at runtime (future work).

## 3. Dependencies & new deps

**New external dependency: yaml-cpp.** Not currently used anywhere in the
tree. Adding as a required dep for `gz-sim-scenario`. Justification:
- Mature, Apache 2.0, packaged everywhere we care about (Ubuntu, Fedora,
  Arch, Homebrew, vcpkg, conda-forge).
- Alternatives (rapidyaml, mini-yaml, writing our own) buy us little.
- Contained: only `gz-sim-scenario` links it; the rest of gz-sim doesn't
  see YAML headers.

CMake:
```cmake
gz_find_package(yaml-cpp VERSION 0.7 REQUIRED)
```
Added to `gz-sim`'s `package.xml` and `README.md` dep list.

## 4. Scenario file schema (v1, frozen)

```yaml
version: 1                                   # required; any other value fails
name: warehouse_100_amr_fast                 # required; used in logs/reports
description: "100 AMRs in warehouse, kinematic, 5x RTF target"  # optional

extends: [ base_warehouse.yaml ]             # optional, array; resolved in order,
                                             # child wins on deep merge

world:                                       # required (unless 'extends' provides)
  sdf: worlds/warehouse.sdf                  # resolved via GZ_SIM_RESOURCE_PATH
  # Future v1.x: 'inline:' for embedded SDF; 'generated:' for procedural worlds

runtime:
  deterministic: false                       # forces serial PreUpdate/Update
  threads:
    post_update: auto                        # int or 'auto'
    parallel_each: true
  log_level: info

physics:
  max_step_size: 0.004
  real_time_factor: 1.0                      # 0.0 = uncapped / as fast as possible
  engines:                                   # sidecar engines by tier (Phase 3)
    kinematic: gz-physics-kinematic          # Phase 2 introduces this plugin
    approximate: dartsim
    accurate: bullet-featherstone

profiles:                                    # named, composable
  kinematic:
    tier: kinematic
    sensors: { rendered: false, update_rate_scale: 0.25 }
  approximate:
    tier: approximate
  accurate:
    tier: accurate
    physics: { max_step_size: 0.001, solver_residual_tol: 1e-5 }
  arm_accurate:
    extends: accurate
    physics: { max_step_size: 0.0005, solver_residual_tol: 1e-6, max_substeps: 16 }

scene:
  generators:                                # procedural spawn
    - template: model://amr_mir250
      count: 100
      name_prefix: amr_
      layout:
        type: grid                           # grid | random | explicit
        origin: [0, 0, 0, 0, 0, 0]           # x y z roll pitch yaw
        spacing: [1.5, 1.5, 0.0]
        rows: 10
  spawn:                                     # individual entities
    - uri: model://ur10_cell
      name: ur10_cell
      pose: [5, 5, 0, 0, 0, 0]
  initial_state:                             # post-spawn overrides
    - entity: ur10_cell::ur10
      joint_positions: { shoulder_lift: -1.57, elbow_joint: 1.57 }

overrides:                                   # first-match-wins, ordered
  - match: { model: "amr_*" }
    profile: kinematic
    plugins:
      mode: replace                          # replace | merge | patch
      list:
        - name: gz-sim-kinematic-motion-system
          params:
            wheel_radius: 0.1
            track: 0.4
            cmd_vel_topic: /${model}/cmd_vel
        - name: gz-sim-odometry-publisher-system
        - name: gz-sim-pose-publisher-system

  - match: { model: "ur10_cell" }
    profile: arm_accurate
    plugins:
      mode: merge
      list:
        - name: gz-sim-joint-state-publisher-system

# Reserved for later phases — parser accepts and ignores with a note:
logging: {}
bridges: {}
record:  {}
```

### 4.1 Match rules

```yaml
match:
  model: "amr_*"            # glob on model name
  link:  "gripper::*"       # optional, v1.x (emits warning in v1 — link-level
                            # fidelity is future work, but schema accepts)
  tags:  ["mobile_base"]    # optional; entities can carry tags via SDF custom
                            # attributes; v1 supports tag matching
  any_of: [...]             # optional; list of sub-matchers, OR-combined
  all_of: [...]             # optional; AND
  not:    {...}             # optional; negation
```

First-match-wins. Within one override rule, `match` is a single predicate;
composition with `any_of` / `all_of` / `not` gives full boolean logic.

### 4.2 Profile resolution

Each entity's effective profile:
1. Default: `"approximate"` (or value of top-level `defaults.profile` if set).
2. Walk `overrides` in order; first rule whose `match` hits the entity
   supplies the profile (and plugin directives).
3. Profile lookup expands `extends` chain → flat set of knobs.
4. The `FidelityTier` component is attached with the tier; full knob set is
   stored in a side map keyed by profile name (not per-entity — we don't
   duplicate).

### 4.3 Plugin resolution

Three modes:
- `replace`: the override's plugin list becomes the entity's plugin list.
  SDF `<plugin>` tags are **discarded** for this entity.
- `merge`: override's list appends to SDF plugins; de-dup by (filename, name).
- `patch`: SDF plugins are the baseline; override's `add` appends and
  `remove` (list of names) subtracts.

Resolution algorithm per entity:
```
sdf_plugins := plugins declared on entity in SDF
resolved    := sdf_plugins
for each override rule that matched entity E, in config-file order:
  case rule.plugins.mode of
    replace: resolved := rule.plugins.list
    merge  : resolved := dedup(resolved ++ rule.plugins.list)
    patch  : resolved := dedup((resolved \ rule.plugins.remove) ++ rule.plugins.add)
materialize plugins in `resolved` via 0d's YAML Configure path
```

`${model}`, `${entity}`, `${name}` are substituted in plugin `params` at
resolution time (pre-materialization), not per-step.

### 4.4 `extends` composition

Scenario-level `extends` is deep-merged:
- Scalars: child wins.
- Maps: recursed.
- Lists: **child replaces** by default. Explicit list-merge uses `<<:` marker
  at the list head (cf. YAML merge key idiom). The common case — overrides
  lists — is "child fully owns"; scenario inheritance is for fidelity
  profile tweaks, not for rewriting rule lists.

Profile-level `extends` (inside `profiles`) is the same deep merge, scoped
to one profile.

## 5. Data model additions to gz-sim

### 5.1 `components::FidelityTier`

```cpp
// gz-sim/include/gz/sim/components/FidelityTier.hh
namespace gz::sim::components {
  enum class FidelityTier : uint8_t { Kinematic, Approximate, Accurate };
  using FidelityTierComponent = Component<FidelityTier, class FidelityTierTag>;
  GZ_SIM_REGISTER_COMPONENT(
    "gz_sim_components.FidelityTier", FidelityTierComponent);
}
```

Attached to every model entity by the scenario resolver. Links inherit
from model via lookup at Phase 2+ consumption time (no per-link component
in v1).

### 5.2 `components::ScenarioProfileName`

Tag component carrying the string profile name for debugging / tooling.
Not read by any system hot path.

### 5.3 `components::ScenarioPluginSet`

Internal: list of resolved plugin descriptors to instantiate. Consumed by
`SystemLoader` during entity setup. Removed after plugins load.

## 6. Library structure

New library: `gz-sim-scenario` (depends on yaml-cpp, `gz-sim`, `sdformat`).

```
gz-sim/include/gz/sim/scenario/
  Scenario.hh             // top-level loaded scenario (opaque to callers)
  ScenarioLoader.hh       // parse + validate + compose 'extends' chain
  ScenarioResolver.hh     // match rules → per-entity profile + plugins
  FidelityProfile.hh      // resolved profile knob set
  ScenarioReport.hh       // resolved-report emitter

gz-sim/src/scenario/
  ScenarioLoader.cc
  ScenarioResolver.cc
  ScenarioReport.cc
  Matcher.{hh,cc}         // glob + any_of/all_of/not predicate tree
  Interpolator.{hh,cc}    // ${var} substitution in plugin params
  YamlToSdf.{hh,cc}       // 0d bridge: YAML::Node → sdf::ElementPtr for the
                          //   default Configure path

gz-sim/test/scenario/
  ScenarioLoader_TEST.cc
  ScenarioResolver_TEST.cc
  Matcher_TEST.cc
  golden/                 // golden-file resolved-report tests
    warehouse_100_amr.yaml
    warehouse_100_amr.resolved.expected.yaml
    ...

gz-sim/examples/scenarios/
  warehouse_100_amr_fast.yaml
  mixed_20_robot.yaml
  ur10_cell_accurate.yaml
```

## 7. CLI integration

### 7.1 Entry point dispatch

Modify [gz-sim's `gz sim` cmd frontend](../../src/cmd/) to dispatch on the
input file extension:
- `.sdf`, `.world` → legacy path (implicit default scenario; log an info
  message about the implicit profile).
- `.yaml`, `.yml` → scenario path.
- otherwise → error.

```
gz sim warehouse_100_amr_fast.yaml              # run a scenario
gz sim warehouse.yaml -c overlay_fast.yaml      # overlay (stacked, last wins)
gz sim world.sdf                                # legacy, unchanged
gz sim scenario resolve warehouse.yaml          # dry-run; print resolved YAML
gz sim scenario convert world.sdf -o scaffold.yaml  # extract SDF plugins
gz sim scenario validate warehouse.yaml         # schema-only check
```

### 7.2 Overlay (`-c`)

Multiple `-c` files stack after `extends` chain; later wins. Equivalent to
appending to `extends` but available without editing files. Useful in CI
for site-level defaults + scenario-specific tweaks.

### 7.3 Resolved-report emission

Every scenario run emits `<output_dir>/gz_scenario_resolved.yaml` describing
the fully-resolved state: final scenario tree, every entity with its
resolved profile name and plugin list, sources of each decision (which
override rule matched). Default `<output_dir>` is `/tmp` on Linux, CWD
otherwise; configurable via `--scenario-report-dir`.

Format (example):
```yaml
scenario:
  name: warehouse_100_amr_fast
  sources: [warehouse_100_amr_fast.yaml, overlay_fast.yaml]
entities:
  - entity: amr_01
    profile: kinematic
    profile_source: "overrides[0] (match: model=amr_*)"
    plugins:
      - { name: gz-sim-kinematic-motion-system, source: "overrides[0]" }
      - { name: gz-sim-odometry-publisher-system, source: "overrides[0]" }
      - { name: gz-sim-pose-publisher-system, source: "overrides[0]" }
  - entity: ur10_cell
    profile: arm_accurate
    profile_source: "overrides[1] (match: model=ur10_cell)"
    plugins:
      - { name: gz-sim-some-plugin, source: "sdf:worlds/warehouse.sdf" }
      - { name: gz-sim-joint-state-publisher-system, source: "overrides[1]" }
```

This output is the anchor for golden-file tests — a small change in
matching logic produces a readable diff.

## 8. Loader flow

```
scenario_path →
  1. Parse YAML → Scenario AST
  2. Resolve `extends` chain (topological, detect cycles)
  3. Apply any `-c` overlays
  4. Validate:
       - schema (required fields, types, enum values)
       - profile references ('profile: kinematic' must exist in profiles map
         or be a built-in)
       - plugin modes and structural validity
       - generator layout sanity (grid: rows divides count)
  5. Load SDF world referenced by scenario.world.sdf
  6. Apply scene.generators → produce entity specs
  7. Apply scene.spawn → more entity specs
  8. Append to (or override) SDF's <include> entities
  9. Pass to SdfEntityCreator → entities materialized in ECM
 10. Apply scene.initial_state overrides
 11. ScenarioResolver: for each entity, walk overrides, assign
       - components::FidelityTier
       - components::ScenarioProfileName
       - components::ScenarioPluginSet
 12. SystemLoader instantiates plugins per entity from ScenarioPluginSet
       (legacy SDF `<plugin>` path is skipped for entities with ScenarioPluginSet
        in 'replace' mode; merged/patched otherwise)
 13. Emit resolved report
 14. Start simulation loop
```

Steps 5–9 are the only invasive change to existing gz-sim loader code —
`SdfEntityCreator` gains a "merge these entity specs with those already in
the SDF" path. This is a small surface.

## 9. Interaction with 0d (YAML Configure)

Plugin params in the scenario are `YAML::Node`s. Phase 1 produces both:
- The raw `YAML::Node` for each plugin — preferred path if the plugin
  exports `GZ_ADD_YAML_CONFIGURE`.
- A synthetic `sdf::ElementPtr` via the `YamlToSdf` bridge — fallback for
  unmodified plugins.

Every in-tree plugin in Phase 1 stays on the synthetic-SDF path. Phase 2+
plugins (`kinematic_motion`, etc.) ship with native YAML configure from
day one.

## 10. `gz sim scenario convert`

Migration tool:
- Input: an SDF world.
- Output: a scaffold scenario that reproduces the SDF's behavior on the
  default `approximate` profile.

Transformation:
- `scenario.world.sdf` = input path (copied, not rewritten).
- For each top-level `<plugin>` in the SDF, emit one `overrides` rule with
  `mode: merge` and the plugin params translated via `SdfToYaml` (the
  inverse of `YamlToSdf`).
- Does not modify the SDF file. Users are expected to subsequently delete
  the `<plugin>` tags from their SDF once satisfied with the scaffold.

This keeps migration **non-destructive** — an SDF that's been run through
`convert` still works on its own.

## 11. Tag matching from SDF

`match.tags: [...]` needs entities to carry tags. SDF 1.x has no native
tag concept, so we repurpose `<model>`'s arbitrary `<custom>` element:

```xml
<model name="amr_01">
  <custom>
    <gz:tags>mobile_base, amr</gz:tags>
  </custom>
  ...
</model>
```

`SdfEntityCreator` parses this into a new `components::Tags` component
(a `std::vector<std::string>`). The matcher reads it. This keeps SDF
agnostic — `<custom>` is explicitly a vendor-extension hook, and other
simulators ignore it.

## 12. Testing plan

- **Unit**: YAML parser, matcher (glob / boolean combinators), interpolator,
  `YamlToSdf` / `SdfToYaml`, `extends` composition.
- **Integration / golden-file**: resolve `examples/scenarios/*.yaml` against
  their referenced SDFs, byte-compare against `*.resolved.expected.yaml`.
  Any change in resolver behavior prints a readable diff.
- **End-to-end**: `gz sim examples/scenarios/warehouse_100_amr_fast.yaml`
  runs, spawns 100 entities, attaches `FidelityTier::Kinematic`, instantiates
  kinematic-motion plugins — verify via a test system that counts entities
  per tier.
- **Legacy path parity**: `gz sim world.sdf` produces the same entities and
  plugins as before Phase 1.
- **`scenario convert` round-trip**: `convert foo.sdf -o foo.yaml` then
  `gz sim foo.yaml` yields identical per-entity plugin sets to
  `gz sim foo.sdf`.
- **Error cases**: cyclic `extends`, missing SDF, unknown profile reference,
  malformed YAML, ambiguous plugin mode — each fails with a clear message
  pointing at the offending file:line.

## 13. Risks

**R1 — SDF-plugin ecosystem friction.** Downstream users with established
SDF `<plugin>` flows may resist the migration. Mitigation: default SDF path
unchanged; scenarios are additive. Deprecation warnings land no earlier
than Phase 5.

**R2 — yaml-cpp version skew.** Older distros (e.g., Ubuntu 20.04) ship
yaml-cpp 0.6. We require 0.7 for `operator=` on `Node` to behave sanely.
Mitigation: document the floor, vendor if needed. Ubuntu 22.04 ships 0.7.

**R3 — matcher surface too rich.** `any_of` / `all_of` / `not` boolean trees
plus globs plus tags may turn into a mini-language users argue about.
Mitigation: document the canonical style in `examples/scenarios/`; deprecate
seldom-used combinators after one release cycle if they're not pulling weight.

**R4 — profile override precedence surprises.** First-match-wins is
simple but means users must order rules correctly. The resolved report
surfaces the decision trail; documentation covers the rules.

**R5 — plugin param interpolation collisions.** `${model}` vs. literal
`$` in some plugin param. Mitigation: only expand `${NAME}` where `NAME`
is a known identifier; leave unknown `$`-expressions verbatim. Warn when
a known variable is expanded inside a string that wasn't explicitly
quoted.

## 14. Milestones

| Week | Deliverable |
|---|---|
| 1 | yaml-cpp integrated; empty `gz-sim-scenario` library builds; parser skeleton. |
| 2 | Full schema parse + validation; `extends` composition; golden-file infra. |
| 3 | Matcher (globs + boolean tree) + tests. `components::FidelityTier` + `Tags`. |
| 4 | ScenarioResolver (profile + plugin resolution); resolved-report emitter. |
| 5 | CLI dispatch (`gz sim x.yaml`, `-c`, `scenario resolve`, `scenario validate`). |
| 6 | SdfEntityCreator integration for `scene.generators` / `scene.spawn` / `initial_state`. |
| 7 | SystemLoader integration for `ScenarioPluginSet`; `scenario convert` tool. |
| 8 | End-to-end tests on the three example scenarios; documentation; Phase 1 tagged. |

One-week slip acceptable; most likely slip point is SystemLoader wiring in
week 7 because it touches plugin loading which has its own legacy quirks.

## 15. What unblocks after Phase 1

- **Phase 2**: implement `gz-physics-kinematic` engine + `kinematic_motion`
  system. Systems can now rely on `FidelityTier` component to know whether
  to process an entity, and can rely on `ScenarioPluginSet` to receive
  their own YAML params.
- **Phase 3**: `Physics.cc` tier-gates engines via `FidelityTier`. All
  entities labeled, physics just reads.
- **Phase 4**: bounded-residual substepping reads profile knobs
  (`solver_residual_tol`, `max_substeps`) set via scenarios.
- **Phase 5+**: all fidelity behaviors pivot on the components written
  here.

After Phase 1, the scenario surface is **data**. Everything subsequent is
reading that data and acting on it.
