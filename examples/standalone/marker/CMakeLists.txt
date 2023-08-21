cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-marker)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-transport12 QUIET REQUIRED)
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

  find_package(gz-common5 REQUIRED)
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  find_package(gz-msgs9 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

  add_executable(marker marker.cc)
  target_link_libraries(marker
    gz-transport${GZ_TRANSPORT_VER}::core
    gz-msgs${GZ_MSGS_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    )
endif()
