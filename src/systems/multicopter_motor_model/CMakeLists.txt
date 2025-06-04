gz_add_system(multicopter-motor-model
  SOURCES
  MulticopterMotorModel.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)
