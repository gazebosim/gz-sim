gz_add_system(joint-position-controller
  SOURCES
    JointPositionController.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)
