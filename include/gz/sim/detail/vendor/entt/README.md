## Vendored Entt

This folder contains the vendored files from the [Entt](https://github.com/skypjack/entt) Entity Component System library.
The `src/entt` folder from the v3.16.0 tag has been copied in, with the exclusion of the `natvis` subfolder that only contains configuration files and the `tools.hpp` header together with the `tools` subfolder that has an `imgui` requirement.

Additionally, the `GAZEBO_MODIFIED_CODE.patch` has been applied to :
  * Fix layering checks in Bazel + Clang and fix compilation with Bazel >= 9.
  * Make entities 64 bits by default.
  * Reduce the size of entity generations and increase the size of entity IDs.
