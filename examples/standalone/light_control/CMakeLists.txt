cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-light-control)

find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

find_package(gz-sim REQUIRED)

add_executable(light_control light_control.cc)
target_link_libraries(light_control
  gz-transport::core
  gz-sim::gz-sim)
