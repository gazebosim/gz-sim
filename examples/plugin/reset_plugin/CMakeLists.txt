cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(ResetPlugins)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(JointPositionRandomizer SHARED JointPositionRandomizer.cc)
target_link_libraries(JointPositionRandomizer
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::core)
