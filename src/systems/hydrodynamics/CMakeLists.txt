gz_add_system(hydrodynamics
  SOURCES
  Hydrodynamics.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::eigen3
)
