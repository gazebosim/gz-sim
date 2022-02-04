cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(RenderingPlugins)

# Common to both plugins
find_package(ignition-rendering3 REQUIRED)

# GUI plugin
set(GUI_PLUGIN RenderingGuiPlugin)

set(CMAKE_AUTOMOC ON)

find_package(ignition-gui3 REQUIRED)

QT5_ADD_RESOURCES(resources_RCC ${GUI_PLUGIN}.qrc)

add_library(${GUI_PLUGIN} SHARED
  ${GUI_PLUGIN}.cc
  ${resources_RCC}
)
target_link_libraries(${GUI_PLUGIN}
  PRIVATE
    ignition-gui3::ignition-gui3
    ignition-rendering3::ignition-rendering3
)

# Server plugin
set(SERVER_PLUGIN RenderingServerPlugin)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo3 REQUIRED)

add_library(${SERVER_PLUGIN} SHARED ${SERVER_PLUGIN}.cc)
set_property(TARGET ${SERVER_PLUGIN} PROPERTY CXX_STANDARD 17)
target_link_libraries(${SERVER_PLUGIN}
  PRIVATE
    ignition-plugin1::ignition-plugin1
    ignition-gazebo3::ignition-gazebo3
    ignition-rendering3::ignition-rendering3
)
