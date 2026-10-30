gz_add_system(joint-position-controller
  SOURCES
    JointPositionController.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)

gz_build_tests(TYPE UNIT
  SOURCES
    JointPositionControllerSystem_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_JointPositionControllerSystem_TEST)
  target_sources(UNIT_JointPositionControllerSystem_TEST PRIVATE JointPositionController.cc)
  target_include_directories(UNIT_JointPositionControllerSystem_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
