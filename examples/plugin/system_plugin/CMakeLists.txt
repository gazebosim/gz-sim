cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake4 REQUIRED)

project(SampleSystem)

find_package(gz-plugin3 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin3_VERSION_MAJOR})

find_package(gz-sim9 REQUIRED)
add_library(SampleSystem SHARED SampleSystem.cc SampleSystem2.cc)
set_property(TARGET SampleSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(SampleSystem
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim9::gz-sim9)
