cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(RenderingPlugins)

# Common to both plugins
find_package(gz-rendering9 REQUIRED)

# GUI plugin
set(GUI_PLUGIN RenderingGuiPlugin)

set(CMAKE_AUTOMOC ON)

find_package(gz-gui9 REQUIRED)

qt_add_resources(resources_RCC ${GUI_PLUGIN}.qrc)

add_library(${GUI_PLUGIN} SHARED
  ${GUI_PLUGIN}.cc
  ${resources_RCC}
)
target_link_libraries(${GUI_PLUGIN}
  PRIVATE
    gz-gui9::gz-gui9
    gz-rendering9::gz-rendering9
)

# Server plugin
set(SERVER_PLUGIN RenderingServerPlugin)

find_package(gz-plugin3 REQUIRED COMPONENTS register)
find_package(gz-sim9 REQUIRED)

add_library(${SERVER_PLUGIN} SHARED ${SERVER_PLUGIN}.cc)
set_property(TARGET ${SERVER_PLUGIN} PROPERTY CXX_STANDARD 17)
target_link_libraries(${SERVER_PLUGIN}
  PRIVATE
    gz-plugin3::gz-plugin3
    gz-sim9::gz-sim9
    gz-rendering9::gz-rendering9
)
