set (gui_sources
  GuiRunner.cc
  TmpIface.cc
)

add_subdirectory(plugins)

set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTORCC ON)

ign_add_component(gui
  SOURCES ${gui_sources} resources/gazebo.qrc
  GET_TARGET_NAME gui_target
  CXX_STANDARD 17)

target_link_libraries(${gui_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-gui${IGN_GUI_VER}::ignition-gui${IGN_GUI_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
)

set (CMAKE_AUTOMOC OFF)
set (CMAKE_AUTORCC OFF)

install(TARGETS ${gui_target} DESTINATION ${IGN_LIB_INSTALL_DIR})
install (FILES gui.config DESTINATION ${IGN_DATA_INSTALL_DIR}/gui)
