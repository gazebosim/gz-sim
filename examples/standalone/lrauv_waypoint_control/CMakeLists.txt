cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-lrauv-waypoint-control)

find_package(gz-transport12 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

add_executable(lrauv_waypoint_control lrauv_waypoint_control.cc)
target_link_libraries(lrauv_waypoint_control
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
