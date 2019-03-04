set (logical_camera_system_sources
  LogicalCamera.cc
)

ign_add_component(logical-camera-system
  SOURCES ${logical_camera_system_sources}
  GET_TARGET_NAME logical_camera_system_target
  CXX_STANDARD 17)

target_link_libraries(${logical_camera_system_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
    ignition-sensors${IGN_SENSORS_VER}::logical_camera
    ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${logical_camera_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
