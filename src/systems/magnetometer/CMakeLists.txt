set (magnetometer_system_sources
  Magnetometer.cc
)

ign_add_component(magnetometer-system
  SOURCES ${magnetometer_system_sources}
  GET_TARGET_NAME magnetometer_system_target
  CXX_STANDARD 17)

target_link_libraries(${magnetometer_system_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${magnetometer_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
