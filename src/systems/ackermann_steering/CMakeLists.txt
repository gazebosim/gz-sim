gz_add_system(ackermann-steering
  SOURCES
    AckermannSteering.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    AckermannSteering_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_AckermannSteering_TEST )
  target_sources(UNIT_AckermannSteering_TEST PRIVATE AckermannSteering.cc)
endif()
