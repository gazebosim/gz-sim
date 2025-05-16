cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake REQUIRED)

project(SampleSystem)

find_package(gz-plugin REQUIRED COMPONENTS register)

find_package(gz-sim REQUIRED)
add_library(SampleSystem SHARED SampleSystem.cc SampleSystem2.cc)
set_property(TARGET SampleSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(SampleSystem
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::gz-sim)
