gz_add_system(joint-controller
  SOURCES
    JointController.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)
