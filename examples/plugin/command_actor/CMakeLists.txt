cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake4 REQUIRED)

project(CommandActor)

find_package(gz-plugin3 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin3_VERSION_MAJOR})

find_package(gz-sim10 REQUIRED)
add_library(CommandActor SHARED CommandActor.cc)
set_property(TARGET CommandActor PROPERTY CXX_STANDARD 17)
target_link_libraries(CommandActor
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim10::gz-sim10)
