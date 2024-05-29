cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-joy-to-twist)

find_package(gz-transport14 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

find_package(sdformat15 REQUIRED)
set(SDF_VER ${sdformat15_VERSION_MAJOR})

add_executable(joy_to_twist joy_to_twist.cc)
target_link_libraries(joy_to_twist
  gz-transport${GZ_TRANSPORT_VER}::core
  sdformat${SDF_VER}::sdformat${SDF_VER}
)
