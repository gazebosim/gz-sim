set (CMAKE_AUTOMOC ON)

# No GUI headers are installed at the moment
set (gui_headers
  GuiRunner.hh
  GuiSystem.hh
  TmpIface.hh
)

QT5_WRAP_CPP(headers_MOC ${gui_headers})
QT5_ADD_RESOURCES(resources_RCC gazebo.qrc)

set (gui_comp_sources
  ${gui_sources}
  ${headers_MOC}
  ${resources_RCC}
)

ign_add_component(gui
  SOURCES ${gui_comp_sources}
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

install(TARGETS ${gui_target} DESTINATION ${IGN_LIB_INSTALL_DIR})

install (FILES gui.config DESTINATION ${IGN_DATA_INSTALL_DIR}/gui)
