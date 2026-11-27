gz_add_system(ackermann-steering
  SOURCES
    AckermannSteering.cc
  TEST_SOURCES
    AckermannSteering_TEST.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)
