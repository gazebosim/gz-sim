cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake REQUIRED)

project(Priority_printer)

gz_find_package(gz-plugin REQUIRED COMPONENTS register)

gz_find_package(gz-sim REQUIRED)

add_library(PriorityPrinter SHARED PriorityPrinter.cc)
set_property(TARGET PriorityPrinter PROPERTY CXX_STANDARD 17)
target_link_libraries(PriorityPrinter
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::gz-sim)
