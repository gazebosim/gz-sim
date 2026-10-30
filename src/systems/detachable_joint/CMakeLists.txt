gz_add_system(detachable-joint
  SOURCES
  DetachableJoint.cc
  TEST_SOURCES
    DetachableJoint_TEST.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)
