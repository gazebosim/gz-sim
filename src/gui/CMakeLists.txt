set (gui_sources
  AboutDialogHandler.cc
  Gui.cc
  GuiEvents.cc
  GuiFileHandler.cc
  GuiRunner.cc
  PathManager.cc
  QuickStartHandler.cc
)

set (gtest_sources
  Gui_TEST.cc
  GuiEvents_TEST.cc
  Gui_clean_exit_TEST.cc
)

add_subdirectory(plugins)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# CMake AUTOMOC does not generate moc_*.cpp files automatically for headers
# located in different directories than the containing .cc file. For Qt header
# files in `include/gz/sim/gui`, we use qt5_wrap_cpp instead. There is
# no need to add entries for Qt header files in `src/gui/`.
qt5_wrap_cpp(gui_sources
  ${PROJECT_SOURCE_DIR}/include/gz/sim/gui/GuiSystem.hh
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

gz_add_component(gui
  SOURCES ${gui_sources} resources/gazebo.qrc
  GET_TARGET_NAME gui_target)

target_link_libraries(${gui_target}
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
)

set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC OFF)

install(TARGETS ${gui_target} DESTINATION ${GZ_LIB_INSTALL_DIR})
install (FILES gui.config DESTINATION ${GZ_DATA_INSTALL_DIR}/gui)
install (FILES playback_gui.config DESTINATION ${GZ_DATA_INSTALL_DIR}/gui)

# Tests
include_directories(${PROJECT_SOURCE_DIR}/test)

gz_build_tests(TYPE UNIT
  SOURCES
    ${gtest_sources}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}
    ${gui_target}
    ${EXTRA_TEST_LIB_DEPS}
    gz-sim${PROJECT_VERSION_MAJOR}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if(TARGET UNIT_Gui_clean_exit_TEST)
  set_tests_properties(UNIT_Gui_clean_exit_TEST PROPERTIES TIMEOUT 300)
endif()
