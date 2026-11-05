gz_add_system(drive-to-pose-controller
  SOURCES
    DriveToPoseController.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    DriveToPoseController_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_DriveToPoseController_TEST)
  target_sources(UNIT_DriveToPoseController_TEST PRIVATE DriveToPoseController.cc)
  target_include_directories(UNIT_DriveToPoseController_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
