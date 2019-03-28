gz_add_system(touchplugin
  SOURCES
    TouchPlugin.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
