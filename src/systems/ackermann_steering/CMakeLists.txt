gz_add_system(ackermann-steering
  SOURCES
    AckermannSteering.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
