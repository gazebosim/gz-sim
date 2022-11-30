cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
# joystick currently works only on linux

project(gz-sim-joystick)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-transport12 QUIET REQUIRED OPTIONAL_COMPONENTS log)
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

  find_package(sdformat13 REQUIRED)
  set(SDF_VER ${sdformat13_VERSION_MAJOR})

  add_executable(joystick joystick.cc)
  target_link_libraries(joystick
    gz-transport${GZ_TRANSPORT_VER}::core
    sdformat${SDF_VER}::sdformat${SDF_VER}
    )
endif()
