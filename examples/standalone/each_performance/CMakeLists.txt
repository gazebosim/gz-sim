cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-each-performance)

find_package(gz-sim8 QUIET REQUIRED)

add_executable(each each.cc)
target_link_libraries(each
  gz-sim8::core)
