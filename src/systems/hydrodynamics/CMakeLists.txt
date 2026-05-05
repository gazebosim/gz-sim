gz_add_system(hydrodynamics
  SOURCES
  Hydrodynamics.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-math::eigen3
)

gz_build_tests(TYPE UNIT
  SOURCES
    Hydrodynamics_TEST.cc
  LIB_DEPS
    gz-math::eigen3
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
