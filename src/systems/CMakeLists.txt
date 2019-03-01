set(IGNITION_GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)

add_subdirectory(altimeter)
add_subdirectory(contact)
add_subdirectory(diff_drive)
add_subdirectory(imu)
add_subdirectory(logical_camera)
add_subdirectory(magnetometer)
add_subdirectory(physics)
add_subdirectory(pose_publisher)
add_subdirectory(sensors)
add_subdirectory(user_commands)
add_subdirectory(scene_broadcaster)
add_subdirectory(touch_plugin)
