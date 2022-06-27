gz_add_system(physics
  SOURCES
    Physics.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-common${GZ_COMMON_VER}::geospatial
    gz-common${GZ_COMMON_VER}::graphics
    gz-common${GZ_COMMON_VER}::profiler
    gz-physics${GZ_PHYSICS_VER}::gz-physics${GZ_PHYSICS_VER}
)

set (gtest_sources
  EntityFeatureMap_TEST.cc
)

gz_build_tests(TYPE UNIT
  SOURCES
  ${gtest_sources}
  LIB_DEPS
  gz-physics${GZ_PHYSICS_VER}::core
)
