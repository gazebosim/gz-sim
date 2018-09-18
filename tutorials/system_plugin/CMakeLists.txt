cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(SampleSystem)

set(IGN_PLUGIN_VER 0)
ign_find_package(ignition-plugin0 REQUIRED COMPONENTS register)

ign_find_package(ignition-gazebo REQUIRED)

add_library(SampleSystem SHARED SampleSystem.cc SampleSystem2.cc)
set_property(TARGET SampleSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(SampleSystem
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo::ignition-gazebo)
