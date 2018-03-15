cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Find the Ignition-Gazebo library
find_package(ignition-gazebo REQUIRED)
find_package(SDFormat REQUIRED)

include_directories(${SDFormat_INCLUDE_DIRS})

add_executable(ball_drop ball_drop.cc)
target_link_libraries(ball_drop ignition-gazebo::ignition-gazebo
                      ${SDFormat_LIBRARIES})
