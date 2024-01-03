gz_add_system(air-pressure
  SOURCES
    AirPressure.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  PRIVATE_LINK_LIBS
    gz-sensors${GZ_SENSORS_VER}::air_pressure
)
