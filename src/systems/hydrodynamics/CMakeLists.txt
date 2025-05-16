gz_add_system(hydrodynamics
  SOURCES
  Hydrodynamics.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-math::eigen3
)
