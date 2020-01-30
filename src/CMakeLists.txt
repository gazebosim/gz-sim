add_subdirectory(rendering)
add_subdirectory(gui)
add_subdirectory(systems)
add_subdirectory(msgs)

set_source_files_properties(
  ${PROTO_PRIVATE_SRC}
  ${PROTO_PRIVATE_HEADERS}
  PROPERTIES GENERATED TRUE
)

# Suppress compiler warnings in generated protobuf C++ code.
set_source_files_properties(
  ${PROTO_PRIVATE_SRC}
  COMPILE_FLAGS -Wno-unused-parameter
)

set(network_sources
  network/NetworkConfig.cc
  network/NetworkManager.cc
  network/NetworkManagerPrimary.cc
  network/NetworkManagerSecondary.cc
  network/PeerInfo.cc
  network/PeerTracker.cc
)

set(gui_sources
  ${gui_sources}
  PARENT_SCOPE
)

ign_add_component(ign SOURCES cmd/ign.cc GET_TARGET_NAME ign_lib_target)
target_link_libraries(${ign_lib_target}
  PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-gazebo${PROJECT_VERSION_MAJOR}
    ignition-gazebo${PROJECT_VERSION_MAJOR}-gui
)

set (sources
  Barrier.cc
  Conversions.cc
  EntityComponentManager.cc
  EventManager.cc
  LevelManager.cc
  Link.cc
  Model.cc
  SdfEntityCreator.cc
  SdfGenerator.cc
  Server.cc
  ServerConfig.cc
  ServerPrivate.cc
  SimulationRunner.cc
  System.cc
  SystemLoader.cc
  Util.cc
  View.cc
  ${PROTO_PRIVATE_SRC}
  ${network_sources}
)

set (gtest_sources
  ${gtest_sources}
  Barrier_TEST.cc
  Component_TEST.cc
  ComponentFactory_TEST.cc
  Conversions_TEST.cc
  EntityComponentManager_TEST.cc
  EventManager_TEST.cc
  ign_TEST.cc
  Link_TEST.cc
  Model_TEST.cc
  SdfEntityCreator_TEST.cc
  SdfGenerator_TEST.cc
  Server_TEST.cc
  SimulationRunner_TEST.cc
  System_TEST.cc
  SystemLoader_TEST.cc
  Util_TEST.cc
  network/NetworkConfig_TEST.cc
  network/PeerTracker_TEST.cc
  network/NetworkManager_TEST.cc
)

# Create the library target
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  ignition-math${IGN_MATH_VER}
  ignition-plugin${IGN_PLUGIN_VER}::core
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-common${IGN_COMMON_VER}::profiler
  ignition-fuel_tools${IGN_FUEL_TOOLS_VER}::ignition-fuel_tools${IGN_FUEL_TOOLS_VER}
  ignition-gui${IGN_GUI_VER}::ignition-gui${IGN_GUI_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  sdformat8::sdformat8
  protobuf::libprotobuf
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::loader
)
if (NOT APPLE)
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE stdc++fs)
endif()

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

add_dependencies(${PROJECT_LIBRARY_TARGET_NAME}
  ignition-gazebo_private_msgs
)

set(IGNITION_GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)

include_directories(${PROJECT_SOURCE_DIR}/test)

# Build the unit tests
ign_build_tests(TYPE UNIT
  SOURCES
    ${gtest_sources}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}
    ${EXTRA_TEST_LIB_DEPS}
    ignition-gazebo${PROJECT_VERSION_MAJOR}
)

# Create the gazebo executable
ign_add_executable(ign-gazebo-exec main.cc)
set_property(TARGET ign-gazebo-exec PROPERTY OUTPUT_NAME ign-gazebo)
target_link_libraries(ign-gazebo-exec PUBLIC ${PROJECT_LIBRARY_TARGET_NAME}
  gflags
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

# Create the server executable
ign_add_executable(ign-gazebo-server server_main.cc)
set_property(TARGET ign-gazebo-server PROPERTY OUTPUT_NAME ign-gazebo-server)
target_link_libraries(ign-gazebo-server PUBLIC ${PROJECT_LIBRARY_TARGET_NAME}
  gflags
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
)

# Create the gui executable
ign_add_executable(ign-gazebo-gui gui_main.cc)
set_property(TARGET ign-gazebo-gui PROPERTY OUTPUT_NAME ign-gazebo-gui)
target_link_libraries(ign-gazebo-gui PUBLIC ${PROJECT_LIBRARY_TARGET_NAME}
  gflags
  ignition-gazebo${PROJECT_VERSION_MAJOR}-gui
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

set_target_properties(ign-gazebo-server
  PROPERTIES VERSION ${PROJECT_VERSION_FULL})
install (TARGETS ign-gazebo-server DESTINATION ${IGN_BIN_INSTALL_DIR})

set_target_properties(ign-gazebo-gui
  PROPERTIES VERSION ${PROJECT_VERSION_FULL})
install (TARGETS ign-gazebo-gui DESTINATION ${IGN_BIN_INSTALL_DIR})

if(NOT WIN32)
  add_subdirectory(cmd)
endif()

