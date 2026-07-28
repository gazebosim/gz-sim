gz_add_system(altimeter
  SOURCES
    Altimeter.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::altimeter
)

gz_build_tests(TYPE UNIT
  SOURCES
    Altimeter_TEST.cc
  LIB_DEPS
    gz-sensors::altimeter
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
