set (levels_system_sources
  Levels.cc
)

ign_add_component(levels-system
  SOURCES ${levels_system_sources}
  GET_TARGET_NAME levels_system_target
  CXX_STANDARD 17)

target_link_libraries(${levels_system_target}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gazebo
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${levels_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
