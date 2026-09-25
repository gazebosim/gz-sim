cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-keyboard)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

  find_package(sdformat REQUIRED)

  find_package(gz-msgs REQUIRED)

  find_package(gz-common REQUIRED)

  add_executable(keyboard keyboard.cc)
  target_link_libraries(keyboard
    gz-transport::core
    sdformat::sdformat
    gz-msgs
    gz-common::gz-common
    stdc++fs
    )
endif()
