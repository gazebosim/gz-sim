# legacy_component_storage (historical reference, not built)

This directory is **not referenced by any build file and is not compiled as part of gz-sim**.
`ComponentStorage<T>` (historically `include/ignition/gazebo/detail/ComponentStorageBase.hh`), the
class [PR #927](https://github.com/gazebosim/gz-sim/pull/927)'s area builds on, no longer exists in
this tree.

Unlike most "target moved" cases in this instance, the underlying issue here isn't just relocated —
it's been independently resolved by a different mechanism. `EntityComponentManager`'s component
storage is now `std::unordered_map<Entity, std::vector<std::unique_ptr<components::BaseComponent>>>`
(`src/EntityComponentManager.cc`): components are heap-allocated and owned by `unique_ptr`, so
growing the per-entity vector only ever moves *pointers*, never the components themselves, and
`RebuildViews()` now has exactly one call site left (a full entity-removal reset) — normal component
creation goes through the incremental `view->MarkEntityToAdd()` path instead. The old
`ComponentStorage<T>`'s `reserve(100)`-and-grow-by-100 `std::vector`, whose reallocation triggered a
full `RebuildViews()` roughly once per 100 components of a type (`O(E)` work, `~E/100` times, giving
`O(E²)` world population), is gone along with the bug it caused.

`ComponentStorageBase_reference.hh`'s `ComponentStorageRef<T>` reconstructs that historical storage
class's algorithm verbatim (from `ignition-gazebo`/`gz-sim` commit `34ac465`, the base this
instance's tooling profiled against), trimmed of the `BaseComponent` virtual-dispatch scaffolding
around it since only the storage/reallocation behavior matters here.

`ComponentStorageBase_ours.hh`'s `ComponentStorageOurs<T>` is the same class backed by `std::deque`
instead: deque never relocates existing elements on growth, so the `expanded` flag `Create()` used
to return (the signal that triggered a full rebuild) is always `false`.

`ComponentStorageBase_TEST.cc` is a standalone check (not wired into CMake, not run by CI) comparing
the two across 8,000 simulated component creations: identical ids and stored values throughout, the
reference's `expanded` flag fires 79 times (matching the tool's own `~E/100` claim almost exactly),
and `ours`'s never fires.
