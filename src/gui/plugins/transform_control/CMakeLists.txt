gz_add_gui_plugin(TransformControl
  SOURCES TransformControl.cc
  QT_HEADERS TransformControl.hh
  PRIVATE_LINK_LIBS
    ${IGNITION-RENDERING_LIBRARIES}
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
)


