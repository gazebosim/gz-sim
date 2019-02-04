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
  ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${log_system_target}
        DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})

# Set install instructions for the sql schema, and configure the build to use it
set(SCHEMA_INSTALL_BASE ${CMAKE_INSTALL_DATAROOTDIR}/ignition/${PROJECT_NAME_LOWER})
install(DIRECTORY sql DESTINATION ${SCHEMA_INSTALL_BASE})
set(SCHEMA_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${SCHEMA_INSTALL_BASE}/sql)
configure_file(build_config.hh.in build_config.hh @ONLY)
