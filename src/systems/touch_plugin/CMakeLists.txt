gz_add_system(touchplugin
  SOURCES
    TouchPlugin.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
