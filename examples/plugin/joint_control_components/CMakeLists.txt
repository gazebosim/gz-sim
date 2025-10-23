cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(JointControlPlugins)

find_package(gz-plugin3 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin3_VERSION_MAJOR})

find_package(gz-sim9 REQUIRED)
set(GZ_SIM_VER ${gz-sim9_VERSION_MAJOR})

add_library(PulseJointVelocityCommand SHARED PulseJointVelocityCommand.cc)
target_link_libraries(PulseJointVelocityCommand
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::core)

add_library(ResetJointVelocityNearPosition SHARED ResetJointVelocityNearPosition.cc)
target_link_libraries(ResetJointVelocityNearPosition
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::core)
