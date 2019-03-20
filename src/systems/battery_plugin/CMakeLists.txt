set (linearbatteryplugin_system_sources
  LinearBatteryPlugin.cc
)

ign_add_component(linearbatteryplugin-system
  SOURCES ${linearbatteryplugin_system_sources}
  GET_TARGET_NAME linearbatteryplugin_system_target
  CXX_STANDARD 17)

target_link_libraries(${linearbatteryplugin_system_target}
  PUBLIC
  ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${linearbatteryplugin_system_target}
        DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
