cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-comms)

find_package(gz-transport12 QUIET REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

add_executable(publisher publisher.cc)
target_link_libraries(publisher
  gz-transport${GZ_TRANSPORT_VER}::core)
