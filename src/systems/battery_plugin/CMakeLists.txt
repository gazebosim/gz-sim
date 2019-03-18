set (battery_plugin_sources
  LinearBatteryPlugin.cc
)

ign_add_component(battery-plugin
  SOURCES ${battery_plugin_sources}
  GET_TARGET_NAME battery_plugin_target
  CXX_STANDARD 17)

target_link_libraries(${battery_plugin_target}
  PUBLIC
  ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${battery_plugin_target}
        DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
