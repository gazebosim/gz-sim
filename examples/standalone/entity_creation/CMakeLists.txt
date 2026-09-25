cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-sim-entity-creation)

find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

add_executable(entity_creation entity_creation.cc)
target_link_libraries(entity_creation
  gz-transport::core)
