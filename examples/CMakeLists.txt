cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find the Ignition_Math library
find_package(ignition_math QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION_MATH_CXX_FLAGS}")
include_directories(${IGNITION_MATH_INCLUDE_DIRS})
link_directories(${IGNITION_MATH_LIBRARY_DIRS})

add_executable(vector2_example vector2_example.cc)
add_executable(triangle_example triangle_example.cc)
