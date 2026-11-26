gz_add_system(joint-state-publisher
  SOURCES
  JointStatePublisher.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)

gz_build_tests(TYPE UNIT
  SOURCES
    JointStatePublisher_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_JointStatePublisher_TEST)
  target_sources(UNIT_JointStatePublisher_TEST PRIVATE JointStatePublisher.cc)
  target_include_directories(UNIT_JointStatePublisher_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
