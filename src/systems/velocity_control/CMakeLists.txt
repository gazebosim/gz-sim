gz_add_system(velocity-control
  SOURCES
    VelocityControl.cc
  TEST_SOURCES
    VelocityControl_TEST.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)
