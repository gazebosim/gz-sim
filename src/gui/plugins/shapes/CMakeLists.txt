gz_add_gui_plugin(Shapes
  SOURCES Shapes.cc
  QT_HEADERS Shapes.hh
  PRIVATE_LINK_LIBS
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
)
