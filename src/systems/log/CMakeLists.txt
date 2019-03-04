set (log_system_sources
  LogRecord.cc
  LogPlayback.cc
)

ign_add_component(log-system
  SOURCES ${log_system_sources}
  GET_TARGET_NAME log_system_target
  CXX_STANDARD 17)

target_link_libraries(${log_system_target}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  # Use ign-transport directly
  ignition-transport${IGN_TRANSPORT_VER}::log
  ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${log_system_target}
        DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
