gz_add_system(wheel-slip
  SOURCES
  WheelSlip.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
