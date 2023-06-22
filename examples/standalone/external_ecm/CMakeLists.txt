cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-external-ecm)

find_package(sdformat13 REQUIRED)
set (SDF_VER ${sdformat13_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)

add_executable(external_ecm external_ecm.cc)
target_link_libraries(external_ecm
  gz-sim7::core
  sdformat${SDF_VER}::sdformat${SDF_VER})
