set (imu_system_sources
  Imu.cc
)

ign_add_component(imu-system
  SOURCES ${imu_system_sources}
  GET_TARGET_NAME imu_system_target
  CXX_STANDARD 17)

target_link_libraries(${imu_system_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${imu_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
