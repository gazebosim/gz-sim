cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-comms)

find_package(gz-transport14 QUIET REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

add_executable(publisher publisher.cc)
target_link_libraries(publisher
  gz-transport${GZ_TRANSPORT_VER}::core)
