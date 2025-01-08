cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake4 REQUIRED)

project(Priority_printer)

gz_find_package(gz-plugin3 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin3_VERSION_MAJOR})

gz_find_package(gz-sim10 REQUIRED)
set(GZ_SIM_VER ${gz-sim10_VERSION_MAJOR})

add_library(PriorityPrinter SHARED PriorityPrinter.cc)
set_property(TARGET PriorityPrinter PROPERTY CXX_STANDARD 17)
target_link_libraries(PriorityPrinter
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
