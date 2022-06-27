gz_add_gui_plugin(VisualizeLidar
  SOURCES VisualizeLidar.cc
  QT_HEADERS VisualizeLidar.hh
  PRIVATE_LINK_LIBS
    gz-rendering${GZ_RENDERING_VER}::core
)
