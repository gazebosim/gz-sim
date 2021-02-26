gz_add_system(physics
  SOURCES
    Physics.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-common${IGN_COMMON_VER}::profiler
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}
)

set (gtest_sources
  EntityFeatureMap_TEST.cc
)

ign_build_tests(TYPE UNIT
  SOURCES
  ${gtest_sources}
  LIB_DEPS
  ignition-physics${IGN_PHYSICS_VER}::core
)
