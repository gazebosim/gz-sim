gz_add_system(multicopter-motor-model
  SOURCES
  MulticopterMotorModel.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport::gz-transport
)
