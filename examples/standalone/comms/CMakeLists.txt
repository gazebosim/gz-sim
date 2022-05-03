cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(ign-gazebo-comms)

find_package(ignition-transport12 QUIET REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport12_VERSION_MAJOR})

add_executable(publisher publisher.cc)
target_link_libraries(publisher
  ignition-transport${IGN_TRANSPORT_VER}::core)
