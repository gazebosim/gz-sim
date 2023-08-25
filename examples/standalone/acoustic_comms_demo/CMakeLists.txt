cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-acoustic-comms-demo)

find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_executable(acoustic_comms_demo acoustic_comms_demo.cc)
target_link_libraries(acoustic_comms_demo
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
