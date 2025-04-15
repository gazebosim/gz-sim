cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-comms)

find_package(gz-transport QUIET REQUIRED)

add_executable(publisher publisher.cc)
target_link_libraries(publisher
  gz-transport::core)
