cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
# joystick currently works only on linux

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(ignition-transport8 QUIET REQUIRED OPTIONAL_COMPONENTS log)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(sdformat8 REQUIRED)
  set(SDF_VER ${sdformat8_VERSION_MAJOR})

  add_executable(joystick joystick.cc)
  target_link_libraries(joystick
    ignition-transport${IGN_TRANSPORT_VER}::core
    sdformat${SDF_VER}::sdformat${SDF_VER}
    )
endif()
