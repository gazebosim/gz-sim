## Vendored Box2D

This folder contains the vendored files from the
[Box2D](https://github.com/erincatto/box2d) 2D physics engine (MIT license, see
`LICENSE`).

* Tag: `v3.1.1`
* Commit: `8c661469c9507d3ad6fbd2fea3f1aa71669c2fe3`

`src/` and `include/` are verbatim copies from that commit, with the exclusion
of:
* `src/CMakeLists.txt`, replaced by `CMakeLists.txt` in this folder (Gazebo
  code, not upstream).
* `src/box2d.natvis`, a Visual Studio debugger visualizer.

Everything else upstream was left out: `samples/`, `test/`, `benchmark/`,
`docs/`, `extern/`, `shared/`, the build scripts and the top-level
`CMakeLists.txt`.

No source file has been modified, so there is no `GAZEBO_MODIFIED_CODE.patch`.

### How gz-sim uses it

The top-level `CMakeLists.txt` has a cache option `GZ_SIM_BOX2D`:

* `AUTO` (default): use a system Box2D 3.1.x
  (`find_package(box2d 3.1...<3.2 CONFIG)`) if found, otherwise build this
  copy. A system Box2D 3.2 or newer is ignored, because the API changes
  between minor releases.
* `SYSTEM`: require a system Box2D 3.1.x. Configure fails with a message for
  3.2 and later.
* `VENDOR`: always build this copy.

Either way the internal target `gz-sim-box2d` is available. Link it privately.
It builds a static, position-independent library with hidden symbol visibility.
The library and its headers are not installed, and no public gz-sim header may
include a Box2D header.

`CMakeLists.txt` mirrors the flags that upstream's CMake files set for GCC and
Clang on Linux:
* C17 with extensions.
* `-ffp-contract=off`, Box2D's setting for deterministic floating-point math.
* Default SIMD (SSE2, NEON), AVX2 off.

Ubuntu and Debian only package Box2D 2.4.1, a different C++ API, so the vendored
copy is what builds there until a distribution packages v3.

### Updating

1. Pick a release tag and note its commit hash.
2. Replace `src/` and `include/` with the upstream copies, delete
   `src/CMakeLists.txt` and `src/box2d.natvis`, and replace `LICENSE`.
3. Compare upstream `src/CMakeLists.txt` and the top-level `CMakeLists.txt`
   with `CMakeLists.txt` here. Carry over new compile flags and definitions.
   The source list is a glob, so new or removed files need no edit.
4. Update the tag and hash above, and move the version range in
   `find_package(box2d 3.1...<3.2 ...)` in the top-level `CMakeLists.txt`
   (and `version.minor` in `INTEGRATION_box2d_dependency`) to the new
   release. The range follows the vendored tag because Box2D changes its API
   between minor releases.
5. Run `INTEGRATION_box2d_dependency` with `GZ_SIM_BOX2D=VENDOR`.
