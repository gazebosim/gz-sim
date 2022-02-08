cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(ResetPlugins)
find_package(ignition-gazebo7 REQUIRED)
add_library(JointPositionRandomizer SHARED joint_position_randomizer.cc)
target_link_libraries(JointPositionRandomizer 
  PRIVATE ignition-gazebo7::core)
