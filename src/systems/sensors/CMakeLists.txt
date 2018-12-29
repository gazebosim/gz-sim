set (sensors_system_sources
  Sensors.cc
  SceneManager.cc
)

ign_add_component(sensors-system
  SOURCES ${sensors_system_sources}
  GET_TARGET_NAME sensors_system_target
  CXX_STANDARD 17)

target_link_libraries(${sensors_system_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
    ignition-sensors${IGN_SENSORS_VER}::camera
    ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${sensors_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
