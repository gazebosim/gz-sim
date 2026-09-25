gz_add_system(logical-camera
  SOURCES
    LogicalCamera.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-sensors::gz-sensors
    gz-sensors::logical_camera
)
