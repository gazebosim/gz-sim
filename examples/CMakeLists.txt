cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find the Ignition-Math library
find_package(ignition-math QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-MATH_CXX_FLAGS}")
include_directories(${IGNITION-MATH_INCLUDE_DIRS})
link_directories(${IGNITION-MATH_LIBRARY_DIRS})

add_executable(vector2_example vector2_example.cc)
add_executable(triangle_example triangle_example.cc)
