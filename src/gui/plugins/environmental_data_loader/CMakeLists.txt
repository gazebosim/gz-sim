gz_add_gui_plugin(EnvironmentalData
  SOURCES EnvironmentalData.cc
  QT_HEADERS EnvironmentalData.hh
  PRIVATE_LINK_LIBS
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
)
