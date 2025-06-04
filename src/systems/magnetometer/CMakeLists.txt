gz_add_system(magnetometer
  SOURCES
    Magnetometer.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::magnetometer
)
