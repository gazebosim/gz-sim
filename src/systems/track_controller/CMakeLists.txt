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
