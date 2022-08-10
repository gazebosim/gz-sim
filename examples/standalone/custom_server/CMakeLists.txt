cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-custom-server)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  add_executable(custom_server custom_server.cc)
  target_link_libraries(custom_server
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    )
endif()
