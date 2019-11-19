cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(ignition-transport8 QUIET REQUIRED OPTIONAL_COMPONENTS log)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(ignition-common3 REQUIRED)
  set(IGN_COMMON_VER ${ignition-common3_VERSION_MAJOR})

  find_package(ignition-msgs5 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})

  add_executable(marker marker.cc)
  target_link_libraries(marker
    ignition-transport${IGN_TRANSPORT_VER}::core
    ignition-msgs${IGN_MSGS_VER}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    )
endif()
