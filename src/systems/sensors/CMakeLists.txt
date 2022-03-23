gz_add_system(sensors
  SOURCES
    Sensors.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    ignition-sensors${IGN_SENSORS_VER}::camera
    ignition-sensors${IGN_SENSORS_VER}::depth_camera
    ignition-sensors${IGN_SENSORS_VER}::gpu_lidar
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
    ignition-sensors${IGN_SENSORS_VER}::lidar
    ignition-sensors${IGN_SENSORS_VER}::rgbd_camera
    ignition-sensors${IGN_SENSORS_VER}::segmentation_camera
    ignition-sensors${IGN_SENSORS_VER}::thermal_camera
    ignition-sensors${IGN_SENSORS_VER}::wide_angle_camera
)

