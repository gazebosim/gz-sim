gz_add_system(opticaltactileplugin
  SOURCES
    OpticalTactilePlugin.cc
    Visualization.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
