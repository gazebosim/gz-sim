cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake3 REQUIRED)

project(CommandActor)

find_package(ignition-plugin2 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin2_VERSION_MAJOR})

find_package(ignition-gazebo7 REQUIRED)
add_library(CommandActor SHARED CommandActor.cc)
set_property(TARGET CommandActor PROPERTY CXX_STANDARD 17)
target_link_libraries(CommandActor
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo7::ignition-gazebo7)
