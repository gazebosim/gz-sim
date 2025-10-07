cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-scene-requester)

find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

add_executable(scene_requester scene_requester.cc)
target_link_libraries(scene_requester
  gz-transport::core)
