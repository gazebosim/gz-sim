gz_add_system(magnetometer
  SOURCES
    Magnetometer.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::magnetometer
)

gz_build_tests(TYPE UNIT
  SOURCES
    Magnetometer_TEST.cc
  LIB_DEPS
    gz-sensors::magnetometer
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_Magnetometer_TEST)
  target_sources(UNIT_Magnetometer_TEST PRIVATE Magnetometer.cc)
endif()
