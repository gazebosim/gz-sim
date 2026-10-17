# Build the vendored Box2D (see README.md in this directory) as a static,
# position-independent library for private use inside gz-sim.
#
# This file is written by Gazebo. Everything else in this directory is a
# verbatim copy of upstream. The flags below are the ones from
# src/CMakeLists.txt and the top-level CMakeLists.txt of the upstream tag
# recorded in README.md that change generated code on GCC and Clang.

file(GLOB GZ_SIM_BOX2D_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

add_library(gz-sim-box2d-vendored STATIC ${GZ_SIM_BOX2D_SOURCES})

target_include_directories(gz-sim-box2d-vendored
  SYSTEM PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Box2D uses C17 for _Static_assert and anonymous unions.
set_target_properties(gz-sim-box2d-vendored PROPERTIES
  C_STANDARD 17
  C_STANDARD_REQUIRED YES
  C_EXTENSIONS YES
  POSITION_INDEPENDENT_CODE ON
  C_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
)

# The library is linked into gz-sim plugins, so keep its symbols out of their
# dynamic symbol tables. A second Box2D in the same process must not clash.
# Default SIMD (SSE2 or NEON) stays on. AVX2 stays off, because the binary has
# to run on any x86_64 machine.

if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  # Box2D's cross-platform determinism needs this
  # (https://box2d.org/posts/2024/08/determinism/). Upstream sets it for the
  # whole project; we set it for this target only.
  target_compile_options(gz-sim-box2d-vendored PRIVATE -ffp-contract=off)
endif()

# Box2D has no warnings-as-errors for us to honor, and its code is not ours to
# fix. Silence warnings for this target only.
if (MSVC)
  target_compile_options(gz-sim-box2d-vendored PRIVATE /W0)
else()
  target_compile_options(gz-sim-box2d-vendored PRIVATE -w)
endif()

if (UNIX)
  target_link_libraries(gz-sim-box2d-vendored PUBLIC m)
endif()
