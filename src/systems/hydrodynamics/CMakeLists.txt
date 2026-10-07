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
<<<<<<< HEAD
    ignition-math${IGN_MATH_VER}::eigen3
=======
    gz-math::eigen3
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
>>>>>>> c46e470 (fix: Set GZ_IP on all types of tests (#4047))
)
