project(CustomSystem)

add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cc)

target_link_libraries(${PROJECT_NAME}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_SENSORS_VER}
  PRIVATE ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
  PRIVATE ignition-gazebo6-rendering
  PRIVATE ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
  PRIVATE custom_rendering_sensor
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ../custom_sensor)
