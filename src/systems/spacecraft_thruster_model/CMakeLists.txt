gz_add_system(spacecraft_thruster_model
  SOURCES
  SpacecraftThrusterModel.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
