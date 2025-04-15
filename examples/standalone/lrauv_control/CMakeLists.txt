cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-lrauv-control)

find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

find_package(gz-sim REQUIRED)

add_executable(lrauv_control lrauv_control.cc)
target_link_libraries(lrauv_control
  gz-transport::core
  gz-sim::gz-sim)
