gz_add_system(drive-to-pose-controller
  SOURCES
    DriveToPoseController.cc
  TEST_SOURCES
    DriveToPoseController_TEST.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)
