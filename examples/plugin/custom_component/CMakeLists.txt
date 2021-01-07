cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(CustomComponentPlugin)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo4 REQUIRED)
add_library(CustomComponentPlugin SHARED
  CustomComponentPlugin.cc
)
set_property(TARGET CustomComponentPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(CustomComponentPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo4::ignition-gazebo4)
