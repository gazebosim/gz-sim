gz_add_system(pose-publisher
  SOURCES
    PosePublisher.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    PosePublisher_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_PosePublisher_TEST)
  target_sources(UNIT_PosePublisher_TEST PRIVATE PosePublisher.cc)
  target_include_directories(UNIT_PosePublisher_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
