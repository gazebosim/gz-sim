gz_add_system(buoyancy-engine
  SOURCES
  BuoyancyEngine.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)
