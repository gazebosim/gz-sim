cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(Hello_world)

ign_find_package(ignition-plugin2 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin2_VERSION_MAJOR})

ign_find_package(ignition-gazebo7 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo7_VERSION_MAJOR})

add_library(HelloWorld SHARED HelloWorld)
set_property(TARGET HelloWorld PROPERTY CXX_STANDARD 17)
target_link_libraries(HelloWorld
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})
