cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

# Find the Ignition-Math library
find_package(ignition-math2 REQUIRED)
include_directories(${IGNITION-MATH_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-MATH_CXX_FLAGS}")

add_executable(vector2_example vector2_example.cc)
target_link_libraries(vector2_example ${IGNITION-MATH_LIBRARIES})

add_executable(triangle_example triangle_example.cc)
target_link_libraries(triangle_example ${IGNITION-MATH_LIBRARIES})
