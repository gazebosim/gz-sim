cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-marker)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-transport QUIET REQUIRED)

  find_package(gz-common REQUIRED)

  find_package(gz-msgs REQUIRED)

  add_executable(marker marker.cc)
  target_link_libraries(marker
    gz-transport::core
    gz-msgs
    gz-common::gz-common
    )
endif()
