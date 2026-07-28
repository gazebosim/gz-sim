gz_add_system(environmental-sensor
  SOURCES
  EnvironmentalSensorSystem.cc
  PUBLIC_LINK_LIBS
    gz-sensors::gz-sensors
    gz-common::gz-common
    gz-math::eigen3
)

gz_build_tests(TYPE UNIT
  SOURCES
  TransformTypes_TEST.cc
  EnvironmentalSensorSystem_TEST.cc
  LIB_DEPS
  gz-math::gz-math
  gz-sensors::gz-sensors
  ENVIRONMENT
  GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
