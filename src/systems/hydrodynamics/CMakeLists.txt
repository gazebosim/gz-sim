gz_add_system(hydrodynamics
  SOURCES
  Hydrodynamics.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

ign_build_tests(TYPE UNIT
  SOURCES
    Hydrodynamics_TEST.cc
  LIB_DEPS
    ignition-math${IGN_MATH_VER}::eigen3
)
