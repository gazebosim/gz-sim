cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
find_package(ignition-cmake2 REQUIRED)
project(CustomSystem)

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-sensors6 REQUIRED)
set(IGN_SENSORS_VER ${ignition-sensors6_VERSION_MAJOR})

find_package(ignition-rendering6 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering6_VERSION_MAJOR})

add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cc)

target_link_libraries(${PROJECT_NAME}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo6
  PRIVATE ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
  PRIVATE ignition-gazebo6-rendering
  PRIVATE ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
  PRIVATE custom_rendering_sensor
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ../custom_sensor)

install (TARGETS CustomSystem DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
