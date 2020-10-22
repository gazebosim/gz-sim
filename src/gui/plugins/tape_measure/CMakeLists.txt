gz_add_gui_plugin(TapeMeasure
  SOURCES TapeMeasure.cc
  QT_HEADERS TapeMeasure.hh
  PRIVATE_LINK_LIBS
    ${IGNITION-RENDERING_LIBRARIES}
)


