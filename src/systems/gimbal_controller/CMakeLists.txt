gz_add_system(gimbalcontrollerplugin
  SOURCES
    GimbalControllerPlugin.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
