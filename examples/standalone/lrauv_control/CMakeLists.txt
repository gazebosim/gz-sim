cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-lrauv-control)

find_package(gz-transport14 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

find_package(gz-sim10 REQUIRED)
set(GZ_SIM_VER ${gz-sim10_VERSION_MAJOR})

add_executable(lrauv_control lrauv_control.cc)
target_link_libraries(lrauv_control
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
