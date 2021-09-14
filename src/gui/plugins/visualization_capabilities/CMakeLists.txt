gz_add_gui_plugin(VisualizationCapabilities
  SOURCES
    VisualizationCapabilities.cc
  QT_HEADERS
    VisualizationCapabilities.hh
  PUBLIC_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)
