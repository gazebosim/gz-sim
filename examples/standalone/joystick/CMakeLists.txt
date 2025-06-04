cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
# joystick currently works only on linux

project(gz-sim-joystick)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

  find_package(sdformat REQUIRED)

  add_executable(joystick joystick.cc)
  target_link_libraries(joystick
    gz-transport::core
    sdformat::sdformat
    )
endif()
