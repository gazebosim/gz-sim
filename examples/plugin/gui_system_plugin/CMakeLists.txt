cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

if(ENABLE_GUI)

  if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
  endif()

  project(GuiSystemPlugin)

  set(CMAKE_AUTOMOC ON)

  find_package(gz-sim REQUIRED COMPONENTS gui)

  QT_ADD_RESOURCES(resources_RCC ${PROJECT_NAME}.qrc)

  add_library(${PROJECT_NAME} SHARED
    ${PROJECT_NAME}.cc
    ${resources_RCC}
  )
  target_link_libraries(${PROJECT_NAME}
    PRIVATE gz-sim::gui
  )

endif()
