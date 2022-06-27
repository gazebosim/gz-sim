gz_add_gui_plugin(ResourceSpawner
  SOURCES ResourceSpawner.cc
  QT_HEADERS ResourceSpawner.hh
  PRIVATE_LINK_LIBS
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
)
