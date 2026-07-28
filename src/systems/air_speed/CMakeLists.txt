gz_add_system(air-speed
  SOURCES
    AirSpeed.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::air_speed
)

gz_build_tests(TYPE UNIT
  SOURCES
    AirSpeed_TEST.cc
  LIB_DEPS
    gz-sensors::air_speed
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
