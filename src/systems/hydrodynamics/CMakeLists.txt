gz_add_system(hydrodynamics
  SOURCES
  Hydrodynamics.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
