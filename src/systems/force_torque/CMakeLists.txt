gz_add_system(forcetorque
  SOURCES
    ForceTorque.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::force_torque
)

gz_build_tests(TYPE UNIT
  SOURCES
    ForceTorque_TEST.cc
  LIB_DEPS
    gz-sensors::force_torque
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_ForceTorque_TEST)
  target_sources(UNIT_ForceTorque_TEST PRIVATE ForceTorque.cc)
endif()
