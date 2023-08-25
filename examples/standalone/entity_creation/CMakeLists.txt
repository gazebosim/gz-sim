cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-entity-creation)

find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

add_executable(entity_creation entity_creation.cc)
target_link_libraries(entity_creation
  gz-transport${GZ_TRANSPORT_VER}::core)
