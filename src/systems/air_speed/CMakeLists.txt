gz_add_system(air-speed
  SOURCES
    AirSpeed.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::air_speed
)
