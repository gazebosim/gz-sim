gz_add_system(logical-camera
  SOURCES
    LogicalCamera.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
    ignition-sensors${IGN_SENSORS_VER}::logical_camera
)

