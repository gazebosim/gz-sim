gz_add_system(joint-trajectory-controller
  SOURCES
    JointTrajectoryController.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)

gz_build_tests(TYPE UNIT
  SOURCES
    JointTrajectoryController_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_JointTrajectoryController_TEST)
  target_sources(UNIT_JointTrajectoryController_TEST PRIVATE JointTrajectoryController.cc)
  target_include_directories(UNIT_JointTrajectoryController_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
