cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-multi-lrauv-race)

find_package(gz-transport14 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

find_package(gz-sim9 REQUIRED)
set(GZ_SIM_VER ${gz-sim9_VERSION_MAJOR})

add_executable(multi_lrauv_race multi_lrauv_race.cc)
target_link_libraries(multi_lrauv_race
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
