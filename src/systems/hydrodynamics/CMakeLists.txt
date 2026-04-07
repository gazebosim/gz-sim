gz_add_system(hydrodynamics
  SOURCES
  Hydrodynamics.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::eigen3
)

gz_build_tests(TYPE UNIT
  SOURCES
    Hydrodynamics_TEST.cc
  LIB_DEPS
    gz-math${GZ_MATH_VER}::eigen3
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
