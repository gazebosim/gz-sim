cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-custom-server)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-sim10 REQUIRED)
  set(GZ_SIM_VER ${gz-sim10_VERSION_MAJOR})

  add_executable(custom_server custom_server.cc)
  target_link_libraries(custom_server
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    )
endif()
