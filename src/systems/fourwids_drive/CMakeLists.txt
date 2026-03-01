gz_add_system(fourwids-drive
  SOURCES
    FourwidsDrive.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
