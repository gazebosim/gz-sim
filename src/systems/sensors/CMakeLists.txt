gz_add_system(sensors
  SOURCES
    Sensors.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
    ignition-sensors${IGN_SENSORS_VER}::camera
    ignition-sensors${IGN_SENSORS_VER}::gpu_lidar
    ignition-sensors${IGN_SENSORS_VER}::depth_camera
    ignition-sensors${IGN_SENSORS_VER}::thermal_camera
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

