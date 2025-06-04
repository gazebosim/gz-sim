cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(ResetPlugins)

find_package(gz-plugin REQUIRED COMPONENTS register)

find_package(gz-sim REQUIRED)

add_library(JointPositionRandomizer SHARED JointPositionRandomizer.cc)
target_link_libraries(JointPositionRandomizer
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::core)
