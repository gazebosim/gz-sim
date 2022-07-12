gz_add_system(linearbatteryplugin
  SOURCES
    LinearBatteryPlugin.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
