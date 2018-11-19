add_subdirectory(gui)
add_subdirectory(systems)

set(gui_sources
  ${gui_sources}
  PARENT_SCOPE
)

set (sources
  Entity.cc
  EntityComponentManager.cc
  EventManager.cc
  Server.cc
  ServerConfig.cc
  ServerPrivate.cc
  SimulationRunner.cc
  System.cc
  SystemManager.cc
  View.cc
)

set (gtest_sources
  ${gtest_sources}
  Conversions_TEST.cc
  EntityComponentManager_TEST.cc
  Entity_TEST.cc
  EventManager_TEST.cc
  Server_TEST.cc
  SimulationRunner_TEST.cc
  System_TEST.cc
  SystemManager_TEST.cc
)

# Create the library target
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  ignition-plugin${IGN_PLUGIN_VER}::core
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gui${IGN_GUI_VER}::ignition-gui${IGN_GUI_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ${SDFormat_LIBRARIES}
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::loader
)

ign_add_component(systems
  SOURCES ${systems_sources}
  GET_TARGET_NAME SYSTEMS_TARGET
  CXX_STANDARD 17)

target_link_libraries(${SYSTEMS_TARGET}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ${SDFormat_LIBRARIES}
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
)

set(IGNITION_GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)
install(TARGETS ${SYSTEMS_TARGET} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})

include_directories(${PROJECT_SOURCE_DIR}/test)

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# Create the gazebo executable
ign_add_executable(ign-gazebo-exec main.cc)
set_property(TARGET ign-gazebo-exec PROPERTY OUTPUT_NAME ign-gazebo)
target_link_libraries(ign-gazebo-exec PUBLIC ${PROJECT_LIBRARY_TARGET_NAME}
  gflags
  ign-gazebo-gui
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
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
