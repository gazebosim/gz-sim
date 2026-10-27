gz_add_system(detachable-joint
  SOURCES
  DetachableJoint.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    DetachableJoint_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_DetachableJoint_TEST)
  target_sources(UNIT_DetachableJoint_TEST PRIVATE DetachableJoint.cc)
  target_include_directories(UNIT_DetachableJoint_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
