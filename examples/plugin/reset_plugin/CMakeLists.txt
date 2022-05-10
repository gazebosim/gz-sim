cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(ResetPlugins)

ign_find_package(ignition-plugin2 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin2_VERSION_MAJOR})

find_package(ignition-gazebo7 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo7_VERSION_MAJOR})

add_library(JointPositionRandomizer SHARED JointPositionRandomizer.cc)
target_link_libraries(JointPositionRandomizer 
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::core)
