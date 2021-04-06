gz_add_gui_plugin(ResourceSpawner
  SOURCES ResourceSpawner.cc
  QT_HEADERS ResourceSpawner.hh
  PRIVATE_LINK_LIBS
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
)
