cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(JointControlPlugins)

find_package(gz-plugin REQUIRED COMPONENTS register)

find_package(gz-sim REQUIRED)

add_library(PulseJointVelocityCommand SHARED PulseJointVelocityCommand.cc)
target_link_libraries(PulseJointVelocityCommand
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::core)

add_library(ResetJointVelocityNearPosition SHARED ResetJointVelocityNearPosition.cc)
target_link_libraries(ResetJointVelocityNearPosition
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::core)
