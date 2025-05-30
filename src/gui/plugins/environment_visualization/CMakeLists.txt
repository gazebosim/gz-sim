gz_add_gui_plugin(EnvironmentVisualization
  SOURCES EnvironmentVisualization.cc
  QT_HEADERS EnvironmentVisualization.hh
  PRIVATE_LINK_LIBS
    gz-common::gz-common
    gz-common::io
    gz-math::gz-math
)
