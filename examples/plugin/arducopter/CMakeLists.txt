
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(arducopter)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo5 REQUIRED)
add_library(ArduCopterPlugin SHARED ArduCopterPlugin)
set_property(TARGET ArduCopterPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(ArduCopterPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo5::ignition-gazebo5)