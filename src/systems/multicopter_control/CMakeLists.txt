gz_add_system(multicopter-control
  SOURCES
  MulticopterVelocityControl.cc
  Common.cc
  LeeVelocityController.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    Eigen3::Eigen
)
