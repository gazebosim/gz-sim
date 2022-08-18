gz_add_system(environmental-data-preload
  SOURCES
    EnvironmentalDataPreload.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-common${GZ_COMMON_VER}::io
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
)
