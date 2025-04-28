add_subdirectory(rendering)
add_subdirectory(gui)
add_subdirectory(systems)

file(GLOB gz_msgs_proto_files
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/sim/private_msgs/*.proto
)

get_target_property(msgs_desc_file
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER} GZ_MSGS_DESC_FILE)

gz_msgs_generate_messages_impl(
  MSGS_GEN_SCRIPT
    ${gz-msgs10_PROTO_GENERATOR_SCRIPT}
  FACTORY_GEN_SCRIPT
    ${gz-msgs10_FACTORY_GENERATOR_SCRIPT}
  GZ_PROTOC_PLUGIN
    ${gz-msgs10_PROTO_GENERATOR_PLUGIN}
  INPUT_PROTOS
    ${gz_msgs_proto_files}
  PROTO_PACKAGE
    "gz.sim.private_msgs"
  PROTO_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/proto
  OUTPUT_DIRECTORY
    ${PROJECT_BINARY_DIR}/gz_msgs_gen
  OUTPUT_SOURCES
    msgs_sources
  OUTPUT_HEADERS
    msgs_headers
  OUTPUT_DETAIL_HEADERS
    msgs_detail_headers
  OUTPUT_PYTHON
    msgs_python
  DEPENDENCY_DESCRIPTIONS ${msgs_desc_file}
)

set(network_sources
  network/NetworkConfig.cc
  network/NetworkManager.cc
  network/NetworkManagerPrimary.cc
  network/NetworkManagerSecondary.cc
  network/PeerInfo.cc
  network/PeerTracker.cc
)

set(comms_sources
  comms/Broker.cc
  comms/ICommsModel.cc
  comms/MsgManager.cc
)

set(gui_sources
  ${gui_sources}
  PARENT_SCOPE
)

set(cli_sources
  gz.cc
  cmd/ModelCommandAPI.cc
)

set(material_sources
  rendering/MaterialParser/MaterialParser.cc
  rendering/MaterialParser/ConfigLoader.cc
)

set (sources
  Actor.cc
  Barrier.cc
  BaseView.cc
  Conversions.cc
  ComponentFactory.cc
  EntityComponentManager.cc
  EntityComponentManagerDiff.cc
  InstallationDirectories.cc
  Joint.cc
  LevelManager.cc
  Light.cc
  Link.cc
  MeshInertiaCalculator.cc
  Model.cc
  Primitives.cc
  SdfEntityCreator.cc
  SdfGenerator.cc
  Sensor.cc
  Server.cc
  ServerConfig.cc
  ServerPrivate.cc
  SimulationRunner.cc
  SystemLoader.cc
  SystemManager.cc
  TestFixture.cc
  Util.cc
  View.cc
  World.cc
  ${network_sources}
  ${comms_sources}
  ${msgs_sources}
  ${material_sources}
)

set (gtest_sources
  ${gtest_sources}
  Actor_TEST.cc
  AddedMass_TEST.cc
  Barrier_TEST.cc
  BaseView_TEST.cc
  ComponentFactory_TEST.cc
  Component_TEST.cc
  Conversions_TEST.cc
  EntityComponentManager_TEST.cc
  EventManager_TEST.cc
  Joint_TEST.cc
  Light_TEST.cc
  Link_TEST.cc
  MeshInertiaCalculator_TEST.cc
  Model_TEST.cc
  Primitives_TEST.cc
  SdfEntityCreator_TEST.cc
  SdfGenerator_TEST.cc
  Sensor_TEST.cc
  ServerConfig_TEST.cc
  Server_TEST.cc
  SimulationRunner_TEST.cc
  SystemLoader_TEST.cc
  SystemManager_TEST.cc
  TestFixture_TEST.cc
  Util_TEST.cc
  World_TEST.cc
  comms/Broker_TEST.cc
  comms/MsgManager_TEST.cc
  network/NetworkConfig_TEST.cc
  network/PeerTracker_TEST.cc
  network/NetworkManager_TEST.cc
)

# gz_TEST and ModelCommandAPI_TEST are not supported with multi config
# CMake generators, see also cmd/CMakeLists.txt
get_property(GENERATOR_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT GENERATOR_IS_MULTI_CONFIG)
  list(APPEND gtest_sources gz_TEST.cc)
endif()


# Tests that require a valid display
set(tests_needing_display
  Server_Rendering_TEST.cc
)

if(NOT GENERATOR_IS_MULTI_CONFIG)
  list(APPEND tests_needing_display ModelCommandAPI_TEST.cc)
endif()

# Add systems that need a valid display here.
# \todo(anyone) Find a way to run these tests with a virtual display such Xvfb
# or Xdummy instead of skipping them
if(VALID_DISPLAY AND VALID_DRI_DISPLAY)
  list(APPEND gtest_sources ${tests_needing_display})
else()
  message(STATUS
    "Skipping these UNIT tests because a valid display was not found:")
  foreach(test ${tests_needing_display})
    message(STATUS " ${test}")
  endforeach(test)
endif()

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${sources} ${gtest_sources} ${cli_sources} COMPILE_FLAGS "/wd4251 /wd4146")
endif()

# CLI
gz_add_component(gz
  SOURCES
    ${cli_sources}
  GET_TARGET_NAME gz_lib_target)
target_link_libraries(${gz_lib_target}
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${PROJECT_VERSION_MAJOR}
    gz-sim${PROJECT_VERSION_MAJOR}-gui
)

# Executable target that runs the GUI without ruby for debugging purposes.
add_executable(runGui cmd/runGui_main.cc)
target_link_libraries(runGui PRIVATE ${gz_lib_target})

# Create the library target
gz_create_core_library(SOURCES ${sources} CXX_STANDARD 17)
gz_add_get_install_prefix_impl(GET_INSTALL_PREFIX_FUNCTION gz::sim::getInstallPrefix
                               GET_INSTALL_PREFIX_HEADER gz/sim/InstallationDirectories.hh
                               OVERRIDE_INSTALL_PREFIX_ENV_VARIABLE GZ_SIM_INSTALL_PREFIX)

set_property(
  SOURCE InstallationDirectories.cc
  PROPERTY COMPILE_DEFINITIONS
  GZ_SIM_GUI_CONFIG_RELATIVE_PATH="${GZ_DATA_INSTALL_DIR}/gui"
  GZ_SIM_SYSTEM_CONFIG_RELATIVE_PATH="${GZ_DATA_INSTALL_DIR}/systems"
  GZ_SIM_SERVER_CONFIG_RELATIVE_PATH="${GZ_DATA_INSTALL_DIR}"
  GZ_SIM_PLUGIN_RELATIVE_INSTALL_DIR="${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins"
  GZ_SIM_GUI_PLUGIN_RELATIVE_INSTALL_DIR="${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/gui"
  GZ_SIM_WORLD_RELATIVE_INSTALL_DIR="${GZ_DATA_INSTALL_DIR}/worlds"
  GZ_SIM_MEDIA_RELATIVE_INSTALL_DIR="${GZ_DATA_INSTALL_DIR}/media"
)
install(FILES "rendering/MaterialParser/gazebo.material" DESTINATION ${GZ_DATA_INSTALL_DIR}/media)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  gz-math${GZ_MATH_VER}
  gz-plugin${GZ_PLUGIN_VER}::core
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-common${GZ_COMMON_VER}::graphics
  gz-common${GZ_COMMON_VER}::profiler
  gz-fuel_tools${GZ_FUEL_TOOLS_VER}::gz-fuel_tools${GZ_FUEL_TOOLS_VER}
  gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
  gz-physics${GZ_PHYSICS_VER}::core
  gz-rendering${GZ_RENDERING_VER}::core
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-transport${GZ_TRANSPORT_VER}::parameters
  sdformat${SDF_VER}::sdformat${SDF_VER}
  protobuf::libprotobuf
  PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::loader
)

if (pybind11_FOUND)
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE pybind11::embed)
  target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PUBLIC -DHAVE_PYBIND11)
endif()

if (UNIX AND NOT APPLE)
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE stdc++fs)
endif()

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/gz_msgs_gen>
)

include_directories(${PROJECT_SOURCE_DIR}/test)

# Build the unit tests
gz_build_tests(TYPE UNIT
  SOURCES
    ${gtest_sources}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}
    ${EXTRA_TEST_LIB_DEPS}
    gz-sim${PROJECT_VERSION_MAJOR}
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

# Some server unit tests require rendering.
if (TARGET UNIT_Server_Rendering_TEST)
 target_link_libraries(UNIT_Server_Rendering_TEST
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  )
endif()

# Command line tests need extra settings
foreach(CMD_TEST
  UNIT_gz_TEST
  UNIT_ModelCommandAPI_TEST)

  if(NOT TARGET ${CMD_TEST})
    continue()
  endif()

  # Running `gz sim` on macOS has problems when run with /usr/bin/ruby
  # due to System Integrity Protection (SIP). Try to find ruby from
  # homebrew as a workaround.
  if (APPLE)
    find_program(BREW_RUBY ruby HINTS /usr/local/opt/ruby/bin)
  endif()

  add_dependencies(${CMD_TEST}
    ${gz_lib_target}
    TestModelSystem
    TestSensorSystem
    TestWorldSystem
  )

  target_compile_definitions(${CMD_TEST} PRIVATE
      "BREW_RUBY=\"${BREW_RUBY} \"")

  target_compile_definitions(${CMD_TEST} PRIVATE
      "GZ_PATH=\"${GZ_TOOLS_PROGRAM}\"")

  set(_env_vars)
  list(APPEND _env_vars "GZ_CONFIG_PATH=${CMAKE_BINARY_DIR}/test/conf")
  list(APPEND _env_vars "GZ_IP=127.0.0.1")
  list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:TestModelSystem>")

  set_tests_properties(${CMD_TEST} PROPERTIES
    ENVIRONMENT "${_env_vars}")

  # On Windows there is no RPATH, so an alternative way for tests for finding .dll libraries
  # in build directory in necessary. For regular tests, the trick is to place all libraries
  # and executables in a common CMAKE_RUNTIME_OUTPUT_DIRECTORY, so that the .dll are found
  # as they are in the same directory where the executable is loaded. For tests that are
  # launched via Ruby, this does not work, so we need to manually add CMAKE_RUNTIME_OUTPUT_DIRECTORY
  # to the PATH. This is done via the ENVIRONMENT_MODIFICATION that is only available
  # since CMake 3.22. However, if an older CMake is used another trick to install the libraries
  # beforehand
  if (WIN32 AND ${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.22")
    set_tests_properties(${CMD_TEST} PROPERTIES
      ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
  endif()

endforeach()

add_subdirectory(cmd)
