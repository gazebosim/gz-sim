cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-acoustic-comms-demo)

find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

find_package(gz-sim REQUIRED)

add_executable(acoustic_comms_demo acoustic_comms_demo.cc)
target_link_libraries(acoustic_comms_demo
  gz-transport::core
  gz-sim::gz-sim)
