gz_add_system(perfect-comms
  SOURCES
  PerfectComms.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
