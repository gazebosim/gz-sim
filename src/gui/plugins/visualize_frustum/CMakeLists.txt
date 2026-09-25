gz_add_gui_plugin(VisualizeFrustum
	SOURCES VisualizeFrustum.cc
	QT_HEADERS VisualizeFrustum.hh
  PRIVATE_LINK_LIBS
    gz-rendering::core
)
