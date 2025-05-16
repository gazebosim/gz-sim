gz_add_system(joint-trajectory-controller
  SOURCES
    JointTrajectoryController.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)
