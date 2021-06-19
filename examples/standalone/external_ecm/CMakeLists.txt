cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-gazebo3 REQUIRED)

add_executable(external_ecm external_ecm.cc)
target_link_libraries(external_ecm
  ignition-gazebo3::core)

