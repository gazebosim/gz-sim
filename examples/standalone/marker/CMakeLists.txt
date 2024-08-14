cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-marker)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-transport14 QUIET REQUIRED)
  set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

  find_package(gz-common6 REQUIRED)
  set(GZ_COMMON_VER ${gz-common6_VERSION_MAJOR})

  find_package(gz-msgs11 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs11_VERSION_MAJOR})

  add_executable(marker marker.cc)
  target_link_libraries(marker
    gz-transport${GZ_TRANSPORT_VER}::core
    gz-msgs${GZ_MSGS_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    )
endif()
