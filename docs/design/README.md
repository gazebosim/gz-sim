# Adjustable-Fidelity Initiative — Design Documents

This directory holds the design docs for the Adjustable-Fidelity fork of
gz-sim. Goals, from the initiating discussion:

- 100+ AMRs in a factory setting at ≥ 5× RTF (workflow sim).
- Dozens of robots at ≥ 1× RTF (medium-fidelity mixed scenes).
- High-accuracy robot-arm dynamics with bounded-residual guarantees
  (may run slower than real-time).
- Same codebase, same SDF worlds — behavior selected per scenario.

## Core concepts

**Fidelity tiers** — `kinematic`, `approximate`, `accurate`. Every entity
is assigned a tier; systems and engines read it to decide how hard to
work on that entity.

**Scenarios** — YAML files that fully describe a runnable simulation.
A scenario references an SDF world and owns everything simulator-specific
(plugins, fidelity, spawn, initial state, runtime knobs). SDF stays
simulator-agnostic; long-term, `<plugin>` tags migrate out of SDF into
scenarios entirely.

**Archetype ECM** — the existing `EntityComponentManager` is re-backed
with an archetype ECS for cache-friendly iteration, deferred mutations
with commit barriers at phase boundaries, and a clean pointer-validity
contract.

**Sidecar engines** — one `gz-physics` engine instance per tier in the
same process. Entities route to exactly one engine by tier. Cross-tier
coupling is one-way static (lower tiers appear as immovable fixtures
to higher tiers).

## Phase order

| Phase | Title | Gate |
|---|---|---|
| [0a](phase-0a-archetype-ecm.md) | Archetype ECS core (standalone) | ≥ 5× iteration perf, 10M entities |
| [0b](phase-0b-ecm-integration.md) | Integrate behind ECM facade | All existing tests green dual-mode |
| [0c](phase-0c-system-port.md) | Port in-tree systems to deferred semantics | Zero cross-phase pointer retention in tree |
| [0d](phase-0d-yaml-configure.md) | YAML Configure ABI | `diff_drive` loads from YAML and SDF |
| [0e](phase-0e-flip-and-upstream.md) | Flip default, remove legacy, RFC upstream | Legacy code deleted, RFC open |
| [1](phase-1-scenarios.md) | Scenarios | Scenario loader, resolver, tools shipped |
| [2](phase-2-kinematic-tier.md) | Kinematic tier + Nav2 compat | `warehouse_100_amr` ≥ 3× RTF, Nav2 round-trip |
| [3](phase-3-sidecar-engines.md) | Sidecar engines | `mixed_20_robot` ≥ 1× RTF |
| [4](phase-4-accurate-tier.md) | Bounded-residual substepping | `ur10_cell_accurate` residual tests |
| [5](phase-5-parallel-systems.md) | Parallel PreUpdate/Update | `warehouse_100_amr` ≥ 5× RTF (FINAL GATE) |
| [6](phase-6-auto-leveling.md) | Auto-leveling for warehouses | 200 AMRs, 1 sim-hour, ≥ 3× RTF |

Future (not documented here):
- Distributed partitioning across machines.
- Dynamic fidelity switching.
- Link-level fidelity overrides.

## Approximate calendar

Sequential wall-clock estimate, small team:
- Phase 0 (a–e): ~20 weeks.
- Phase 1: 4 weeks.
- Phase 2: 8 weeks.
- Phase 3: 7 weeks.
- Phase 4: 6 weeks (parallelizable with 3 after week 3).
- Phase 5: 8 weeks.
- Phase 6: 6 weeks.

Critical path: 0 → 1 → 2 → 3 → 5. 5× gate lands ~12 months after 0a
starts, assuming sequential execution and no major slips.

## Out-of-scope / future

- Distributed multi-machine sim (Phase 7).
- Dynamic fidelity switching (arm cell transitions kinematic→accurate
  when a task begins).
- Link-level fidelity overrides.
- GPU offload of any phase.
- Python-system parallelization.

## Status

All phases currently in design / not started. Phase 0a is the immediate
next actionable piece of work.
