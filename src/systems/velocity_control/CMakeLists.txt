gz_add_system(velocity-control
  SOURCES
    VelocityControl.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
