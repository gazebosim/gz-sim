cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-each-performance)

find_package(gz-sim QUIET REQUIRED)

add_executable(each each.cc)
target_link_libraries(each
  gz-sim::core)
