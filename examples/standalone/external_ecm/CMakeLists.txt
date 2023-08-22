cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-sim-external-ecm)

find_package(gz-sim8 REQUIRED)

add_executable(external_ecm external_ecm.cc)
target_link_libraries(external_ecm
  gz-sim8::core)
