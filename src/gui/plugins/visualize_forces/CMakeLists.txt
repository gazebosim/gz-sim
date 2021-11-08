gz_add_gui_plugin(VisualizeForces
  SOURCES
    VisualizeForces.cc
  QT_HEADERS
    VisualizeForces.hh
  PUBLIC_LINK_LIBS
    ${IGNITION-RENDERING_LIBRARIES}
)
