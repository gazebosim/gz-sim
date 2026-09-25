cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake REQUIRED)

project(CommandActor)

find_package(gz-plugin REQUIRED COMPONENTS register)

find_package(gz-sim REQUIRED)
add_library(CommandActor SHARED CommandActor.cc)
set_property(TARGET CommandActor PROPERTY CXX_STANDARD 17)
target_link_libraries(CommandActor
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::gz-sim)
