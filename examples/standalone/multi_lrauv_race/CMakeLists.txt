cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-multi-lrauv-race)

find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_executable(multi_lrauv_race multi_lrauv_race.cc)
target_link_libraries(multi_lrauv_race
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
