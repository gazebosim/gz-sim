gz_add_system(air-pressure
  SOURCES
    AirPressure.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::air_pressure
)
