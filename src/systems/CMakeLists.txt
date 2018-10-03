set (systems_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/Null.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/SceneBroadcaster.cc
  PARENT_SCOPE
)

set (gtest_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/SceneBroadcaster_TEST.cc
  PARENT_SCOPE
)

# TODO(addisu) Restructure the directory structure of systems, maybe each
# system has it's own directory?
set (physics_system_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/Physics.cc
)

ign_add_component(physics-system
  SOURCES ${physics_system_sources}
  GET_TARGET_NAME physics_system_target
  CXX_STANDARD 17)

target_link_libraries(${physics_system_target}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}
  ignition-gazebo
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
  ignition-plugin${IGN_PLUGIN_VER}::loader
  # TODO(addisu) Need SDFormat?
)

target_compile_definitions(${physics_system_target} PRIVATE
  "dartsim_plugin_LIB=\"$<TARGET_FILE:ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}-dartsim-plugin>\"")

set(IGNITION_GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)
install(TARGETS ${physics_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
