gz_add_system(apply-joint-force
  SOURCES
  ApplyJointForce.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)

gz_build_tests(TYPE UNIT
  SOURCES
    ApplyJointForce_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_ApplyJointForce_TEST)
  target_sources(UNIT_ApplyJointForce_TEST PRIVATE ApplyJointForce.cc)
  target_include_directories(UNIT_ApplyJointForce_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

