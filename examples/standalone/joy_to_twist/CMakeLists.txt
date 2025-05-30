cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-joy-to-twist)

find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

find_package(sdformat REQUIRED)

add_executable(joy_to_twist joy_to_twist.cc)
target_link_libraries(joy_to_twist
  gz-transport::core
  sdformat::sdformat
)
