cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(realsense_camera)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

# Find ignition-sensors
ign_find_package(ignition-sensors5 REQUIRED
)
set(IGN_SENSORS_VER ${ignition-sensors5_VERSION_MAJOR})

# Find ignition-rendering
ign_find_package(ignition-rendering5 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering5_VERSION_MAJOR})

find_package(ignition-gazebo5 REQUIRED)
add_library(RealSensePlugin SHARED RealSensePlugin)
set_property(TARGET RealSensePlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(RealSensePlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo5::ignition-gazebo5
  PRIVATE ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
  PRIVATE ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER})
