gz_add_system(imu
  SOURCES
    Imu.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::imu
)
