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
if(NOT MSVC)
  set_source_files_properties(
    ${PROTO_PRIVATE_SRC}
    COMPILE_FLAGS -Wno-unused-parameter
  )
endif()

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

ign_add_component(ign
  SOURCES
    ign.cc
    cmd/ModelCommandAPI.cc
  GET_TARGET_NAME ign_lib_target)
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
  LevelManager.cc
  Link.cc
  Model.cc
  SdfEntityCreator.cc
  SdfGenerator.cc
  Server.cc
  ServerConfig.cc
  ServerPrivate.cc
  SimulationRunner.cc
  SystemLoader.cc
  TestFixture.cc
  Util.cc
  View.cc
  World.cc
  cmd/ModelCommandAPI.cc
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
  ModelCommandAPI_TEST.cc
  SdfEntityCreator_TEST.cc
  SdfGenerator_TEST.cc
  Server_TEST.cc
  ServerConfig_TEST.cc
  SimulationRunner_TEST.cc
  System_TEST.cc
  SystemLoader_TEST.cc
  TestFixture_TEST.cc
  Util_TEST.cc
  World_TEST.cc
  network/NetworkConfig_TEST.cc
  network/PeerTracker_TEST.cc
  network/NetworkManager_TEST.cc
)

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${sources} ${gtest_sources} COMPILE_FLAGS "/wd4251 /wd4146")
endif()

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
  sdformat${SDF_VER}::sdformat${SDF_VER}
  protobuf::libprotobuf
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::loader
)
if (UNIX AND NOT APPLE)
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

# Command line tests need extra settings
foreach(CMD_TEST
  UNIT_ign_TEST
  UNIT_ModelCommandAPI_TEST)

  if(NOT TARGET ${CMD_TEST})
    continue()
  endif()

  # Running `ign gazebo` on macOS has problems when run with /usr/bin/ruby
  # due to System Integrity Protection (SIP). Try to find ruby from
  # homebrew as a workaround.
  if (APPLE)
    find_program(BREW_RUBY ruby HINTS /usr/local/opt/ruby/bin)
  endif()

  add_dependencies(${CMD_TEST}
    ${ign_lib_target}
    TestModelSystem
    TestSensorSystem
    TestWorldSystem
  )

  target_compile_definitions(${CMD_TEST} PRIVATE
      "BREW_RUBY=\"${BREW_RUBY} \"")

  target_compile_definitions(${CMD_TEST} PRIVATE
      "IGN_PATH=\"${IGNITION-TOOLS_BINARY_DIRS}\"")

  set(_env_vars)
  list(APPEND _env_vars "IGN_CONFIG_PATH=${CMAKE_BINARY_DIR}/test/conf")
  list(APPEND _env_vars "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:TestModelSystem>")

  set_tests_properties(${CMD_TEST} PROPERTIES
    ENVIRONMENT "${_env_vars}")

endforeach()

if(NOT WIN32)
  add_subdirectory(cmd)
endif()

