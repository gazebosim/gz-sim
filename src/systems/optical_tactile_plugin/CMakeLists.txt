gz_add_system(opticaltactileplugin
  SOURCES
    OpticalTactilePlugin.cc
    Visualization.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)