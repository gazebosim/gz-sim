gz_add_gui_plugin(EnvironmentLoader
  SOURCES EnvironmentLoader.cc
  QT_HEADERS EnvironmentLoader.hh
  PRIVATE_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-common${GZ_COMMON_VER}::io
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
)
