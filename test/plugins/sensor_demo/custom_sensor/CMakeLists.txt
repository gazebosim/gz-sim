cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(custom_rendering_sensor)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-sensors6 REQUIRED)
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-sensors6-rendering REQUIRED)

add_library(${PROJECT_NAME} SHARED CustomRenderingSensor.cc)

target_link_libraries(${PROJECT_NAME}
  PUBLIC 
    ignition-sensors6::ignition-sensors6
    ignition-rendering6::ignition-rendering6
    ignition-msgs8::ignition-msgs8
    ignition-transport11::ignition-transport11
    ignition-common4::ignition-common4
    ignition-math6::ignition-math6
    ignition-sensors6::ignition-sensors6-rendering
  )

install (TARGETS custom_rendering_sensor DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
install (TARGETS custom_rendering_sensor DESTINATION ${IGN_LIB_INSTALL_DIR})
