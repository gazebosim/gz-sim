cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-scene-requester)

find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

add_executable(scene_requester scene_requester.cc)
target_link_libraries(scene_requester
  gz-transport${GZ_TRANSPORT_VER}::core)
