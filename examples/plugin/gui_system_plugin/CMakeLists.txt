cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(GuiSystemPlugin)

set(CMAKE_AUTOMOC ON)

find_package(gz-sim9 REQUIRED COMPONENTS gui)

QT5_ADD_RESOURCES(resources_RCC ${PROJECT_NAME}.qrc)

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_NAME}.cc
  ${resources_RCC}
)
target_link_libraries(${PROJECT_NAME}
  PRIVATE gz-sim9::gui
)
