gz_add_gui_plugin(ViewControl
  SOURCES ViewControl.cc
  QT_HEADERS ViewControl.hh
  PRIVATE_LINK_LIBS
    ${IGNITION-RENDERING_LIBRARIES}
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
)


