## Vendored Entt

This folder contains the vendored files from the [Entt](https://github.com/skypjack/entt) Entity Component System library.
The `src/entt` folder from the v3.16.0 tag has been copied in, with the exclusion of:

* The `natvis` subfolder that only contains configuration files.
* The `tools.hpp` header together with the `tools` subfolder that has an `imgui` requirement.
* The `meta` subfolder that introduced compile time warnings and is unused.
* The `entt.hpp` and `fwd.hpp` headers that are unused and would have needed to be patched because of the above removal.

Additionally, the `GAZEBO_MODIFIED_CODE.patch` has been applied to:
  * Fix layering checks in Bazel + Clang and fix compilation with Bazel >= 9.
