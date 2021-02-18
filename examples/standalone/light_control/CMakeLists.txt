cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-transport9 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport9_VERSION_MAJOR})

find_package(ignition-gazebo4 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo4_VERSION_MAJOR})

add_executable(light_control light_control.cc)
target_link_libraries(light_control
  ignition-transport${IGN_TRANSPORT_VER}::core
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})
