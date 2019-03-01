set (touchplugin_system_sources
  TouchPlugin.cc
)

ign_add_component(touchplugin-system
  SOURCES ${touchplugin_system_sources}
  GET_TARGET_NAME touchplugin_system_target
  CXX_STANDARD 17)

target_link_libraries(${touchplugin_system_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${touchplugin_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})

