cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
# joystick currently works only on linux

project(gz-sim-joystick)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)
  set(GZ_TRANSPORT_VER ${gz-transport15_VERSION_MAJOR})

  find_package(sdformat15 REQUIRED)
  set(SDF_VER ${sdformat15_VERSION_MAJOR})

  add_executable(joystick joystick.cc)
  target_link_libraries(joystick
    gz-transport${GZ_TRANSPORT_VER}::core
    sdformat${SDF_VER}::sdformat${SDF_VER}
    )
endif()
