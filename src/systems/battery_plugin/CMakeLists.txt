gz_add_system(linearbatteryplugin
  SOURCES
    LinearBatteryPlugin.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
