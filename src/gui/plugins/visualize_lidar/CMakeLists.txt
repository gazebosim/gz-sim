gz_add_gui_plugin(VisualizeLidar
  SOURCES VisualizeLidar.cc
  QT_HEADERS VisualizeLidar.hh
  PRIVATE_LINK_LIBS
    ${IGNITION-RENDERING_LIBRARIES}
)
