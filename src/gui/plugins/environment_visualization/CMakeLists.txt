gz_add_gui_plugin(EnvironmentVisualization
  SOURCES EnvironmentVisualization.cc
  QT_HEADERS EnvironmentVisualization.hh
  PRIVATE_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-common${GZ_COMMON_VER}::io
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
)
