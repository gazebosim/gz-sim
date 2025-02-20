gz_add_system(reversible-multicopter-motor-model
  SOURCES
    ReversibleMulticopterMotorModel.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)
