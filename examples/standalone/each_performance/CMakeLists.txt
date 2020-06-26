cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-gazebo4 QUIET REQUIRED)

add_executable(each each.cc)
target_link_libraries(each
  ignition-gazebo4::core)

