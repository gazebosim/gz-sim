gz_add_system(lighter_than_air_dynamics
  SOURCES
  LighterThanAirDynamics.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::eigen3
)
