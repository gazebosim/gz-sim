gz_add_system(spacecraft-thruster-model
  SOURCES
  SpacecraftThrusterModel.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
