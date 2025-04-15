gz_add_system(buoyancy-engine
  SOURCES
  BuoyancyEngine.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport::gz-transport
)
