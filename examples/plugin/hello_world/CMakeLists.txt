cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake REQUIRED)

project(Hello_world)

gz_find_package(gz-plugin REQUIRED COMPONENTS register)

gz_find_package(gz-sim REQUIRED)

add_library(HelloWorld SHARED HelloWorld.cc)
set_property(TARGET HelloWorld PROPERTY CXX_STANDARD 17)
target_link_libraries(HelloWorld
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::gz-sim)
