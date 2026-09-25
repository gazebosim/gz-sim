cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-multi-lrauv-race)

find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

find_package(gz-sim REQUIRED)

add_executable(multi_lrauv_race multi_lrauv_race.cc)
target_link_libraries(multi_lrauv_race
  gz-transport::core
  gz-sim::gz-sim)
