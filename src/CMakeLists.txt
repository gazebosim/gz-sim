add_subdirectory(gui)

set(gui_sources
  ${gui_sources}
  PARENT_SCOPE
)

set (sources
  Entity.cc
  System.cc
  TestSystem.cc
)

set (gtest_sources
  Entity_TEST.cc
  System_TEST.cc
)

# Create the library target
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 14)
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  ign-gazebo-gui
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gui${IGN_GUI_VER}::ignition-gui${IGN_GUI_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# Create the gazebo executable
ign_add_executable(ign-gazebo-exec main.cc)
set_property(TARGET ign-gazebo-exec PROPERTY OUTPUT_NAME ign-gazebo)
target_link_libraries(ign-gazebo-exec PUBLIC ${PROJECT_LIBRARY_TARGET_NAME}
  gflags
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gui${IGN_GUI_VER}::ignition-gui${IGN_GUI_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

# Switch to ign_install_executable when it is fixed (https://bitbucket.org/ignitionrobotics/ign-cmake/issues/29)
set_target_properties(ign-gazebo-exec
  PROPERTIES VERSION ${PROJECT_VERSION_FULL})
install (TARGETS ign-gazebo-exec DESTINATION ${IGN_BIN_INSTALL_DIR})
