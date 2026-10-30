gz_add_system(joint-controller
  SOURCES
    JointController.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)

gz_build_tests(TYPE UNIT
  SOURCES
    JointController_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_JointController_TEST)
  target_sources(UNIT_JointController_TEST PRIVATE JointController.cc)
  target_include_directories(UNIT_JointController_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
