gz_add_system(conveyor-belt
  SOURCES
    ConveyorBelt.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)
