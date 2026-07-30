gz_add_system(air-pressure
  SOURCES
    AirPressure.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::air_pressure
)

gz_build_tests(TYPE UNIT
  SOURCES
    AirPressure_TEST.cc
  LIB_DEPS
    gz-sensors::air_pressure
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

target_sources(UNIT_AirPressure_TEST PRIVATE AirPressure.cc)
