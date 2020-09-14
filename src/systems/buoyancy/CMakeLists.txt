gz_add_system(buoyancy
  SOURCES
  Buoyancy.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
