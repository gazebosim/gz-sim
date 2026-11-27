gz_add_system(odometry-publisher
  SOURCES
    OdometryPublisher.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    OdometryPublisher_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_OdometryPublisher_TEST)
  target_sources(UNIT_OdometryPublisher_TEST PRIVATE OdometryPublisher.cc)
  target_include_directories(UNIT_OdometryPublisher_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
