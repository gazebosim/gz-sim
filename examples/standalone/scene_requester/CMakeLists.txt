cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-scene-requester)

find_package(gz-transport14 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

add_executable(scene_requester scene_requester.cc)
target_link_libraries(scene_requester
  gz-transport${GZ_TRANSPORT_VER}::core)
