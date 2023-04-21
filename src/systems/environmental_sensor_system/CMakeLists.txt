gz_add_system(environmental-sensor
  SOURCES
  EnvironmentalSensorSystem.cc
  PUBLIC_LINK_LIBS
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::eigen3
)

gz_build_tests(TYPE UNIT
  SOURCES
  TransformTypes_TEST.cc
  LIB_DEPS
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  ENVIRONMENT
  GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
