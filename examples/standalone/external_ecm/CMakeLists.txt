cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(ign-gazebo-external-ecm)

find_package(ignition-gazebo5 REQUIRED)

add_executable(external_ecm external_ecm.cc)
target_link_libraries(external_ecm
  ignition-gazebo5::core)
