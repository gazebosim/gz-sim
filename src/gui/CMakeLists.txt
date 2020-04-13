set (gui_sources
  Gui.cc
  GuiFileHandler.cc
  GuiRunner.cc
  TmpIface.cc
)

add_subdirectory(plugins)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# CMake AUTOMOC does not generate moc_*.cpp files automatically for headers
# located in different directories than the containing .cc file. For Qt header
# files in `include/ignition/gazebo/gui`, we use qt5_wrap_cpp instead. There is
# no need to add entries for Qt header files in `src/gui/`.
qt5_wrap_cpp(gui_sources
  ${PROJECT_SOURCE_DIR}/include/ignition/gazebo/gui/GuiSystem.hh
  # TODO (anyone) Remove when GuiRunner.hh is moved to `src`
  ${PROJECT_SOURCE_DIR}/include/ignition/gazebo/gui/GuiRunner.hh
  # TODO (anyone) Remove when TmpIface.hh is moved to `src`
  ${PROJECT_SOURCE_DIR}/include/ignition/gazebo/gui/TmpIface.hh
)

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

set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC OFF)

install(TARGETS ${gui_target} DESTINATION ${IGN_LIB_INSTALL_DIR})
install (FILES gui.config DESTINATION ${IGN_DATA_INSTALL_DIR}/gui)
