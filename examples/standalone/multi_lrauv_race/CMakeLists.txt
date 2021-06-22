cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-transport11 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

add_executable(multi_lrauv_race multi_lrauv_race.cc)
target_link_libraries(multi_lrauv_race
  ignition-transport${IGN_TRANSPORT_VER}::core
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})
