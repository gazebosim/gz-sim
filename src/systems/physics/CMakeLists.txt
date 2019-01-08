set (physics_system_sources
  Physics.cc
)

ign_add_component(physics-system
  SOURCES ${physics_system_sources}
  GET_TARGET_NAME physics_system_target
  CXX_STANDARD 17)

target_link_libraries(${physics_system_target}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-common${IGN_COMMON_VER}::profiler
  ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}
  ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
)

target_compile_definitions(${physics_system_target} PRIVATE
  "dartsim_plugin_LIB=\"$<TARGET_FILE:ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}-dartsim-plugin>\"")

install(TARGETS ${physics_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
