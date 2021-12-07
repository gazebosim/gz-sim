cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-transport12 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport12_VERSION_MAJOR})

add_executable(scene_requester scene_requester.cc)
target_link_libraries(scene_requester
  ignition-transport${IGN_TRANSPORT_VER}::core)

