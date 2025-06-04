cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake REQUIRED)

project(CustomComponentPlugin)

find_package(gz-plugin REQUIRED COMPONENTS register)

find_package(gz-sim REQUIRED)
add_library(CustomComponentPlugin SHARED
  CustomComponentPlugin.cc
)
set_property(TARGET CustomComponentPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(CustomComponentPlugin
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::gz-sim)
