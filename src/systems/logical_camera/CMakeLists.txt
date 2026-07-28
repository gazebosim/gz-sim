gz_add_system(logical-camera
  SOURCES
    LogicalCamera.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-sensors::gz-sensors
    gz-sensors::logical_camera
)

gz_build_tests(TYPE UNIT
  SOURCES
    LogicalCamera_TEST.cc
  LIB_DEPS
    gz-sensors::logical_camera
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
