cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-keyboard)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)
  set(GZ_TRANSPORT_VER ${gz-transport15_VERSION_MAJOR})

  find_package(sdformat15 REQUIRED)
  set(SDF_VER ${sdformat15_VERSION_MAJOR})

  find_package(gz-msgs11 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs11_VERSION_MAJOR})

  find_package(gz-common6 REQUIRED)
  set(GZ_COMMON_VER ${gz-common6_VERSION_MAJOR})

  add_executable(keyboard keyboard.cc)
  target_link_libraries(keyboard
    gz-transport${GZ_TRANSPORT_VER}::core
    sdformat${SDF_VER}::sdformat${SDF_VER}
    gz-msgs${GZ_MSGS_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    stdc++fs
    )
endif()
