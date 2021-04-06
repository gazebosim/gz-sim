gz_add_gui_plugin(Shapes
  SOURCES Shapes.cc
  QT_HEADERS Shapes.hh
  PRIVATE_LINK_LIBS
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
)
