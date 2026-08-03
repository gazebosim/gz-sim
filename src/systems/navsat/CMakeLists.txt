gz_add_system(navsat
  SOURCES
    NavSat.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::navsat
)

gz_build_tests(TYPE UNIT
  SOURCES
    NavSat_TEST.cc
  LIB_DEPS
    gz-sensors::navsat
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_NavSat_TEST)
  target_sources(UNIT_NavSat_TEST PRIVATE NavSat.cc)
endif()
