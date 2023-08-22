cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-joy-to-twist)

find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

find_package(sdformat14 REQUIRED)
set(SDF_VER ${sdformat14_VERSION_MAJOR})

add_executable(joy_to_twist joy_to_twist.cc)
target_link_libraries(joy_to_twist
  gz-transport${GZ_TRANSPORT_VER}::core
  sdformat${SDF_VER}::sdformat${SDF_VER}
)
