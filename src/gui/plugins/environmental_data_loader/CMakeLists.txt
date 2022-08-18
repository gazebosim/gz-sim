gz_add_gui_plugin(EnvironmentalDataLoader
  SOURCES EnvironmentalDataLoader.cc
  QT_HEADERS EnvironmentalDataLoader.hh
  PRIVATE_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-common${GZ_COMMON_VER}::io
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
)
