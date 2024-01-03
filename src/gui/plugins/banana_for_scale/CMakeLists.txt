gz_add_gui_plugin(BananaForScale
  SOURCES BananaForScale.cc
  QT_HEADERS BananaForScale.hh
  PRIVATE_LINK_LIBS
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
)
