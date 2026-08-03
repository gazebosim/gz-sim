gz_add_system(track-controller
  SOURCES
    TrackController.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-physics::gz-physics
    gz-msgs::gz-msgs
    gz-math::gz-math
    gz-plugin::gz-plugin
    gz-transport::gz-transport
)

gz_build_tests(TYPE UNIT
  SOURCES
    TrackController_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_TrackController_TEST)
  target_sources(UNIT_TrackController_TEST PRIVATE TrackController.cc)
  target_include_directories(UNIT_TrackController_TEST PRIVATE
    ${PROJECT_SOURCE_DIR}/test)
endif()
