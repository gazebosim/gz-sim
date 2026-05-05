# Archetype ECS architecture

This document describes the archetype-based Entity Component System
(ECS) that backs Gazebo Sim's `EntityComponentManager` (ECM) when
built with `-DGZ_SIM_ARCHETYPE_ECM=ON`.

The original ECM stores components in per-type hash maps keyed by
entity. That model is simple but pays a random-access cost on every
iteration: `Each<Pose, Velocity>` does one map lookup per entity per
component, scattering reads across the heap. As world sizes grow into
the hundreds of robots, iteration becomes a dominant cost.

The archetype ECS reorganizes the same data so that iteration is a
linear walk through cache-friendly arrays. The public
`EntityComponentManager` API and SDF surface are unchanged; the
storage underneath is replaced.

## High-level data model

An **Entity** is a 64-bit handle that carries an index plus a
generation counter. The generation increments when the entity is
destroyed, so stale handles can be detected on lookup.

Every entity belongs to exactly one **archetype**. An archetype is
identified by the *set* of component types its entities carry — for
example, the archetype `{Pose, LinearVelocity, Name}` contains every
entity that has those three components and no others. Two entities
with the same component set live in the same archetype.

Each archetype owns a list of fixed-size memory blocks called
**chunks** (16 KiB by default). A chunk lays its entities out as
parallel arrays (struct-of-arrays), one column per component type:

```
Entity (index, gen) ──► EntityIndex ──► EntityRecord {archetype, chunk, row}
                                                │
                                                ▼
                        Archetype ──► [Chunk, Chunk, Chunk, ...]
                           │
                           └─► columns: [Pose[]], [Velocity[]], [Name[]], ...
                                         (one SoA array per ComponentTypeId)
```

Iterating `Each<Pose, Velocity>` becomes three nested loops:

```cpp
for (archetype_id : query.matching_archetypes) {
  for (chunk : archetype.chunks) {
    Entity*    e  = chunk.entities();
    Pose*      p  = chunk.column<Pose>();
    Velocity*  v  = chunk.column<Velocity>();
    for (row = 0; row < chunk.count; ++row) {
      callback(e[row], p[row], v[row]);
    }
  }
}
```

The inner loop is pure array indexing — predictable, vectorizable,
and friendly to the data cache.

## Core data structures

### Archetype

An archetype owns:

- A sorted, deduplicated vector of `ComponentTypeId`s.
- Precomputed per-column byte offsets within a chunk (alignment-
  aware: each column starts on `max(16, alignof(T))`).
- A list of chunks, last-may-be-non-full.
- Lazy edges to neighboring archetypes — the archetype obtained by
  adding or removing one component type. Filled in on first
  transition, then cached.

Type sets are sorted so archetype equality is a vector compare, and
archetype lookup is a hash on the sorted vector.

### Chunk

A chunk is a 16 KiB aligned allocation that holds up to N rows for a
specific archetype. The byte layout is:

```
[ Entity[capacity] ] [ pad ] [ Pose[capacity] ] [ pad ] [ Velocity[capacity] ] [ pad ] ...
```

Entity is always column 0, so per-row entity lookup is a constant
offset regardless of archetype. Trailing padding aligns each column
to its type's natural boundary.

The 16 KiB size is a tradeoff between amortizing per-chunk loop
overhead and staying within L2 on typical cores. It can be overridden
via `GZ_SIM_ECM_CHUNK_SIZE` for benchmarking.

### EntityIndex

Maps every live `Entity` to its current `(archetype, chunk, row)`.
Inserts, lookups, and erases are all O(1). The index uses a free-list
of destroyed slots and bumps the generation counter on destroy so
stale handles fail fast on lookup.

### Archetype graph

When you add or remove a component on an entity, its archetype
changes. The graph caches each archetype's immediate neighbors:

```
Archetype{Pose, Velocity}
   │
   │  add Name           ┌──► Archetype{Pose, Velocity, Name}
   ├────────────────────►│
   │                     │
   │  remove Velocity    └──► Archetype{Pose}
   ├────────────────────►
```

Edges populate lazily and stay cached, so a "tag this entity" pattern
costs O(1) per transition after warm-up.

### Query

A `Query` is a compiled list of required and excluded component
types. The first call against a fresh archetype graph compiles the
match list (which archetype IDs satisfy the query). Subsequent calls
reuse the cached list, refreshing only when the graph version
counter advances (i.e., when a new archetype was created since the
last query run).

Queries are typically scoped to a single `Each<...>` call site and
cached automatically — system code doesn't construct them by hand.

### Command buffer

Mutations during a system phase (`PreUpdate`, `Update`, `PostUpdate`)
are recorded into a per-phase `CommandBuffer`:

```cpp
enum class CommandKind : uint8_t {
  CreateEntity, DestroyEntity,
  AddComponent, RemoveComponent, SetComponent
};

struct Command {
  CommandKind     kind;
  Entity          entity;
  ComponentTypeId type;
  uint32_t        payload_offset;  // into the buffer's blob arena
  uint32_t        payload_size;
};
```

`World::Commit()` drains the buffer, applies operations in insertion
order, and only then mutates the entity index, archetypes, and chunks.

Each worker thread owns its own command buffer, so parallel
iteration with mutation is a first-class pattern — no locks on the
hot path. Commit merges thread-local buffers in a deterministic order
(main thread first, then workers by insertion).

### Change tracking

The ECM has long supported `EachNew`, `EachRemoved`, and
`EachChanged`. The archetype ECS preserves those semantics:

- **New** — Each archetype tracks a "newly created" bitset per chunk.
  When an entity transitions into an archetype, the destination
  row's bit is set. `ClearNewlyCreatedEntities()` zeroes the bitsets.
- **Removed** — `World::Remove` appends a removal descriptor
  `(ArchetypeId, ComponentTypeId, saved_value)` to a per-step
  `removal_log`. `EachRemoved<T>` walks the log. The saved value
  preserves the component's pre-removal state for consumers
  (scene_broadcaster, logging) that need it.
- **Modified** — One dirty bit per (row, column), packed as
  `uint64_t` words appended to each chunk. `Modify<T>(entity)` and
  the convenience write paths set the bit. `EachChanged<T>` skips
  rows whose bit is clear. `SetAllComponentsUnchanged()` does a
  bulk zero.

## Operations

### Create entity with a component pack

1. Sort and deduplicate the pack's `ComponentTypeId`s.
2. Look up — or lazily create — the archetype for that set.
3. Acquire a row in the archetype's last chunk (allocate a new
   chunk if needed).
4. Placement-construct each component into its column.
5. Allocate an entity slot, wire the index, write the entity into
   column 0.
6. Return the `Entity` handle.

### Add component

1. Locate the entity's current archetype `A` via the index.
2. Follow the cached `add_target` edge to the destination
   archetype `B`, or compute it on first transition.
3. Reserve a row in `B`. Move every type in `A ∩ B` across; move-
   construct the new type into its column.
4. Erase the source row in `A` via swap-with-last; patch the
   victim's index slot.
5. Update the entity's index entry to point at the new row in `B`.

Cost is O(k) where k is the number of components on the entity.
Trivially-relocatable types use `memcpy` for the column migration.

### Remove component / destroy entity

Symmetric to Add. Destroy bumps the entity's generation in its slot
and pushes the slot onto the free list.

### Each / EachParallel

`Each<T...>` iterates query-matching archetypes, then their chunks,
then rows — column pointers are hoisted before the inner loop so
the body is straight-line array access.

`EachParallel<T...>` shards the work-unit list `(archetype_id,
chunk_idx)` across a `gz::common::WorkerPool` owned by the world.
Each worker writes its mutations to a thread-local command buffer;
all buffers merge at the next `Commit`. No cross-worker
synchronization on the hot path.

## Pointer-validity contract

A pointer or reference returned by `EntityComponentManager::Component<T>(entity)`
or by an `Each` callback is valid for the **current system phase**
(`PreUpdate`, `Update`, or `PostUpdate`). It is invalidated by the
next phase commit.

In practice:

- Reading a component pointer inside an `Each` callback or anywhere
  in a single `PreUpdate`/`Update`/`PostUpdate` method is safe —
  the legacy behavior is preserved.
- Storing an ECM pointer in a system member field across phases is
  not safe. Cache the `Entity` handle instead and re-read in each
  phase.

```cpp
// OK — pointers are local to the callback.
void MySystem::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  _ecm.Each<components::Pose, components::LinearVelocity>(
      [&](const Entity &, const components::Pose *_pose,
          const components::LinearVelocity *_vel) -> bool
      {
        DoWork(*_pose, *_vel);
        return true;
      });
}

// NOT OK — poseComp is invalidated at the next phase commit.
class MySystem
{
  private: components::Pose *poseComp = nullptr;
};
```

Phase boundaries on the ECM are explicit:

```cpp
ecm.BeginPhase();
//   ... systems run ...
ecm.CommitPhase();
```

`SimulationRunner` brackets each system phase with these calls, so
typical system code never invokes them directly.

## Mutation visibility

The archetype core supports strict deferred mutation: a
`CreateComponent` issued during `PreUpdate` is not visible to a
subsequent `Component<T>` read in the same phase — both apply at
the next `CommitPhase()`. Inside `EachParallel`, deferral is
mandatory (each worker writes to its thread-local buffer).

The `EntityComponentManager` facade currently runs in **immediate-
mutation** mode for compatibility with in-tree systems that read
back a freshly-created component within the same phase. New code
should avoid that pattern — combine the create with the data
(`CreateComponent(e, T{value})`) rather than `CreateComponent(e, T{}); ptr->Set(value)` —
because a future release will switch the facade to strict deferred
semantics. See `Migration.md` for the transition plan.

## Memory management

- Chunks are 16 KiB aligned allocations (`posix_memalign` /
  `_aligned_malloc`).
- Archetypes retain their chunks for the world's lifetime. Empty
  archetypes stay allocated; this is normal in archetype ECS and
  the per-archetype overhead is small (a few KB).
- Component values live as inline slices of the chunk's storage —
  there is no per-component heap allocation.
- The command buffer's payload arena is reset to size 0 at each
  commit; capacity is retained.

## Thread safety

- `Each` is safe to call concurrently from multiple reader threads.
- `EachParallel` is the intended primitive for parallel work. Body
  code may read freely; mutations route through the thread-local
  command buffer.
- `Commit` is single-threaded — `SimulationRunner` calls it between
  phases.
- Component-type registration is mutex-guarded (cold path).
- Determinism: `EachParallel`'s chunk assignment is deterministic
  given a fixed worker count, and command-buffer merge order is
  `(thread_id, insertion_order)`. Step-for-step determinism across
  machines requires pinning the worker count.

## Build flag

The archetype backend is selected at configure time:

```sh
cmake -DGZ_SIM_ARCHETYPE_ECM=ON ..
```

When ON, `libgz-sim.so` links against the archetype core
(`gz-sim-ecs`, a static library compiled from `src/ecs/`) and the
ECM facade in `src/EntityComponentManagerArchetype.cc` is selected
in place of the legacy `src/EntityComponentManager.cc`. The two
implementations live side-by-side; the build system picks one.

There are no `#ifdef` branches inside either implementation file —
each is a complete, linear implementation. The single conditional
include sits at the bottom of the public header, selecting which
template-bodies header to pull in.

## Performance

Reference measurements from the standalone `bench_vs_ecm` benchmark
(in-tree `components::Pose` + `components::LinearVelocity`,
`Each<const Pose, Velocity>` reads `Pose.x` and writes
`Velocity.y = f(Pose.x)`):

| Entity count | Legacy ECM | Archetype | Speedup |
|---:|---:|---:|---:|
| 100 000 | 56.8 ns/entity | 1.14 ns/entity | **49.87×** |
| 1 000 000 | 20.7 ns/entity | 2.53 ns/entity | **8.21×** |

The per-entity gap shrinks at larger N because both storage models
become memory-bandwidth bound, but the archetype stays comfortably
above the design's 5× iteration target.

`EachParallel` hits 6.89× speedup at 8 workers on the 10M-entity
microbench (0.86× of linear). Drop-off at 16 workers on a 12-core
host is expected — bandwidth-bound trivial lambdas don't keep that
many cores fed; compute-bound system bodies (integration, control
loops) scale closer to linear.

### Entity creation is slower

Per-call entity creation is *slower* under the archetype backend —
about 2.6× slower than the legacy ECM at 1M entities. Each
`Create` / `CreateComponent` pair has to:

- sort and deduplicate the component-type pack so the archetype
  hash key is canonical,
- look up the destination archetype (and on first sighting of a
  type set, allocate the archetype, compute the column layout,
  size the dirty-bit bitset, and bump the query-cache version
  counter),
- reserve a row, then placement-construct each component at its
  precomputed column offset.

The legacy ECM does almost none of that — it hash-inserts each
component into its own per-type map. The archetype trades that
simpler write path for the cache-friendly read path that wins back
8–50× during iteration.

Why this is acceptable in practice:

- World load and level load happen once. Iteration runs every
  PreUpdate / Update / PostUpdate — typically ~1000 times per
  simulated second.
- The expensive parts amortize. Layout computation and dirty-bit
  allocation only fire on the *first* create into a new archetype.
  The 200th wheeled robot lands in an existing archetype and pays
  only the per-row append cost.
- Real spawn patterns batch — SDF model loads and level loads
  produce groups of entities sharing the same component set, which
  is exactly the case where the archetype lookup is a hash hit.

## File layout

Public headers (the archetype core's surface):

```
include/gz/sim/ecs/
  World.hh
  Entity.hh
  Query.hh
  ComponentTypeRegistry.hh
  CommandBuffer.hh
```

Internal headers (implementation detail, not part of the public
API):

```
include/gz/sim/ecs/detail/
  Archetype.hh
  ArchetypeGraph.hh
  Chunk.hh
  ColumnLayout.hh
  EntityIndex.hh
  WorldImpl.hh
```

Implementation:

```
src/ecs/
  World.cc
  Archetype.cc
  ArchetypeGraph.cc
  Chunk.cc
  EntityIndex.cc
  Query.cc
  ComponentTypeRegistry.cc
  CommandBuffer.cc
```

The ECM facade that bridges the existing public API to the archetype
core lives in
`src/EntityComponentManagerArchetype.cc` and the parallel template
header at `include/gz/sim/detail/EntityComponentManagerArchetype.hh`.

## What stays the same

- The `EntityComponentManager` public API.
- Component type registration — `GZ_SIM_REGISTER_COMPONENT` works
  unchanged. The archetype core reuses the existing
  `gz::sim::ComponentTypeId` (`uint64_t`) and the same hashing
  algorithm `Factory::Register` already uses, so component IDs are
  stable across backends.
- SDF surface — worlds, models, plugins all load identically.
- Wire formats — `msgs::SerializedState` and
  `msgs::SerializedStateMap` round-trip identically; downstream
  consumers (scene_broadcaster, network secondaries, state log
  recorder) see no difference.
- System callback signatures — `Configure`, `PreUpdate`, `Update`,
  `PostUpdate`, `Reset` are all unchanged.

## What changes for plugin authors

The only user-visible behavioral shift is the **pointer-validity
contract** described above: a `Component<T>(e)` pointer is valid for
the current phase only. The vast majority of in-tree systems already
follow this pattern; cross-phase pointer caching was technically
undefined under the legacy ECM as well, but may have worked in
practice.

`Migration.md` carries the migration recipe and recommended /
anti-pattern examples.

## References

- The implementation lives under `src/ecs/` and `include/gz/sim/ecs/`.
- The archetype-backed ECM facade is in
  `src/EntityComponentManagerArchetype.cc`.
- Phase-by-phase design notes — useful for understanding the
  motivation behind specific choices — live under `docs/design/`
  alongside the rest of the gz-sim design corpus.
