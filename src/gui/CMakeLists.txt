set (gui_sources
  AboutDialogHandler.cc
  Gui.cc
  GuiFileHandler.cc
  GuiRunner.cc
  PathManager.cc
)

set (gtest_sources
  Gui_TEST.cc
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
)

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${gui_sources} ${gtest_sources}
      COMPILE_FLAGS "/wd4251 /wd4146")
endif()

ign_add_component(gui
  SOURCES ${gui_sources} resources/gazebo.qrc
  GET_TARGET_NAME gui_target
  CXX_STANDARD 17)

target_link_libraries(${gui_target}
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-gui${IGN_GUI_VER}::ignition-gui${IGN_GUI_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
)

set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC OFF)

install(TARGETS ${gui_target} DESTINATION ${IGN_LIB_INSTALL_DIR})
install (FILES gui.config DESTINATION ${IGN_DATA_INSTALL_DIR}/gui)
install (FILES playback_gui.config DESTINATION ${IGN_DATA_INSTALL_DIR}/gui)

# Tests
include_directories(${PROJECT_SOURCE_DIR}/test)

ign_build_tests(TYPE UNIT
  SOURCES
    ${gtest_sources}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}
    ${gui_target}
    ${EXTRA_TEST_LIB_DEPS}
    ignition-gazebo${PROJECT_VERSION_MAJOR}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
)
