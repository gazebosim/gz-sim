gz_add_gui_plugin(BananaForScale
  SOURCES BananaForScale.cc
  QT_HEADERS BananaForScale.hh
  PRIVATE_LINK_LIBS
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
)
