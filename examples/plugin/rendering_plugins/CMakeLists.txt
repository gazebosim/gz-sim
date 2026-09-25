cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(RenderingPlugins)

# Common to both plugins
find_package(gz-rendering REQUIRED)

# GUI plugin
set(GUI_PLUGIN RenderingGuiPlugin)

set(CMAKE_AUTOMOC ON)

find_package(gz-gui REQUIRED)

QT_ADD_RESOURCES(resources_RCC ${GUI_PLUGIN}.qrc)

add_library(${GUI_PLUGIN} SHARED
  ${GUI_PLUGIN}.cc
  ${resources_RCC}
)
target_link_libraries(${GUI_PLUGIN}
  PRIVATE
    gz-gui::gz-gui
    gz-rendering::gz-rendering
)

# Server plugin
set(SERVER_PLUGIN RenderingServerPlugin)

find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz-sim REQUIRED)

add_library(${SERVER_PLUGIN} SHARED ${SERVER_PLUGIN}.cc)
set_property(TARGET ${SERVER_PLUGIN} PROPERTY CXX_STANDARD 17)
target_link_libraries(${SERVER_PLUGIN}
  PRIVATE
    gz-plugin::gz-plugin
    gz-sim::gz-sim
    gz-rendering::gz-rendering
)
