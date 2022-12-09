gz_add_system(environmental-sensor
  SOURCES
  EnvironmentalSensorSystem.cc
  PUBLIC_LINK_LIBS
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::eigen3
)
