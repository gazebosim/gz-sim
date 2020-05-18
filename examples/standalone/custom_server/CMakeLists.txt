cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_COMMON_VER ${ignition-gazebo3_VERSION_MAJOR})

  add_executable(custom_server custom_server.cc)
  target_link_libraries(custom_server
    ignition-gazebo${IGN_COMMON_VER}::ignition-gazebo${IGN_COMMON_VER}
    )
endif()
