gz_add_gui_plugin(EnvironmentLoader
  SOURCES EnvironmentLoader.cc
  QT_HEADERS EnvironmentLoader.hh
  PRIVATE_LINK_LIBS
    gz-common::gz-common
    gz-common::io
    gz-math::gz-math
)
