cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake REQUIRED)

project(OdometerSystem)

gz_find_package(gz-plugin REQUIRED COMPONENTS register)

gz_find_package(gz-sim REQUIRED)

find_package(gz-sensors REQUIRED)

# Fetch the custom sensor example from gz-sensors
# Users won't commonly use this to fetch their sensors. The sensor may be part
# of the system's CMake project, or installed from another project, etc...
include(FetchContent)
FetchContent_Declare(
  sensors_clone
  GIT_REPOSITORY https://github.com/gazebosim/gz-sensors
  GIT_TAG main
)
FetchContent_Populate(sensors_clone)
add_subdirectory(${sensors_clone_SOURCE_DIR}/examples/custom_sensor ${sensors_clone_BINARY_DIR})

add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    gz-plugin::gz-plugin
    gz-sim::gz-sim
    gz-sensors::gz-sensors
    odometer
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${sensors_clone_SOURCE_DIR}/examples/custom_sensor)
