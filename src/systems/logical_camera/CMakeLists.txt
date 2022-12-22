gz_add_system(logical-camera
  SOURCES
    LogicalCamera.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    gz-sensors${GZ_SENSORS_VER}::logical_camera
)
