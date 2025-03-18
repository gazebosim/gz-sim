cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-light-control)

find_package(gz-transport14 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

find_package(gz-sim REQUIRED)

add_executable(light_control light_control.cc)
target_link_libraries(light_control
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-sim::gz-sim)
