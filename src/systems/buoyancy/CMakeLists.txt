gz_add_system(buoyancy
  SOURCES
  Buoyancy.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::graphics
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
