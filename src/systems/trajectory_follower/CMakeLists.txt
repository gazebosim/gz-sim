gz_add_system(trajectory-follower
  SOURCES
  TrajectoryFollower.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    TrajectoryFollower_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=$(CMAKE_INSTALL_PREFIX)
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_TrajectoryFollower_TEST)
 target_sources(UNIT_TrajectoryFollower_TEST PRIVATE TrajectoryFollower.cc)
 target_include_directories(UNIT_TrajectoryFollower_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
