cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake4 REQUIRED)

project(CustomComponentPlugin)

find_package(gz-plugin3 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin3_VERSION_MAJOR})

find_package(gz-sim10 REQUIRED)
add_library(CustomComponentPlugin SHARED
  CustomComponentPlugin.cc
)
set_property(TARGET CustomComponentPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(CustomComponentPlugin
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim10::gz-sim10)
