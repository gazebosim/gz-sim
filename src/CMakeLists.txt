#============================================================================
# Optional shared precompiled header (see GZ_SIM_ENABLE_PCH in the top-level
# CMakeLists.txt). Building the PCH once in a dedicated object library and
# having every consumer REUSE_FROM it keeps the (large) precompiled header on
# disk and in memory a single time instead of once per target. This must be
# defined before add_subdirectory(systems) so the system plugins can reuse it.
#============================================================================
if(GZ_SIM_ENABLE_PCH)
  add_library(gz-sim-pch OBJECT pch_provider.cc)

  # The consumers (core library and plugins) are position-independent shared
  # objects; REUSE_FROM requires the provider's PIC setting to match, and
  # object libraries default to PIC off.
  set_target_properties(gz-sim-pch PROPERTIES POSITION_INDEPENDENT_CODE ON)

  # Inherit the include directories of the dependencies whose headers the PCH
  # pulls in, plus gz-sim's own source and generated public headers. The core
  # library target does not exist yet, so its dependencies are listed directly.
  target_link_libraries(gz-sim-pch PRIVATE
    gz-math
    gz-common::gz-common
    gz-common::profiler
    gz-transport::gz-transport
    gz-msgs::gz-msgs
    gz-plugin::core
    gz-plugin::register
    sdformat::sdformat
    protobuf::libprotobuf
  )
  target_include_directories(gz-sim-pch PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_BINARY_DIR}/include
  )

  # Contents are ordered by parse cost: protobuf/gz-msgs and sdf dominate,
  # followed by the gz core libraries and the C++ standard library, with
  # EntityComponentManager.hh last. EnTT is deliberately absent: its parse
  # self-time is negligible (see the entt-improvements compile-time analysis),
  # and the component headers are already declare-only after R1.
  target_precompile_headers(gz-sim-pch PRIVATE
    # C++ standard library (most-included headers first)
    <memory>
    <string>
    <vector>
    <unordered_map>
    <unordered_set>
    <map>
    <set>
    <utility>
    <algorithm>
    <mutex>
    <chrono>
    <optional>
    <limits>
    <functional>
    # protobuf / gz-msgs runtime -- the dominant per-TU parse cost
    <gz/msgs/Utility.hh>
    # SDFormat
    <sdf/sdf.hh>
    # gz core libraries (hot headers shared by most translation units)
    <gz/math/Pose3.hh>
    <gz/math/Vector3.hh>
    <gz/math/Helpers.hh>
    <gz/transport/Node.hh>
    <gz/common/Profiler.hh>
    <gz/common/Console.hh>
    <gz/common/Util.hh>
    # NOTE: gz/plugin/Register.hh is intentionally NOT in the PCH. It emits a
    # GzPluginHook symbol in every translation unit that includes it and is
    # meant to be included exactly once per plugin; force-including it into a
    # shared PCH causes multiple-definition link errors in multi-TU targets.
    # gz-sim stable public headers
    <gz/sim/System.hh>
    <gz/sim/config.hh>
    # EntityComponentManager last (pulls in Factory.hh and the vendored EnTT)
    <gz/sim/EntityComponentManager.hh>
  )
endif()

add_subdirectory(rendering)

if(ENABLE_GUI)
  add_subdirectory(gui)
endif()

add_subdirectory(systems)

file(GLOB gz_msgs_proto_files
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/sim/private_msgs/*.proto
)

get_target_property(msgs_desc_file
  gz-msgs::gz-msgs GZ_MSGS_DESC_FILE)

gz_msgs_generate_messages_impl(
  MSGS_GEN_SCRIPT
    ${gz-msgs_PROTO_GENERATOR_SCRIPT}
  FACTORY_GEN_SCRIPT
    ${gz-msgs_FACTORY_GENERATOR_SCRIPT}
  GZ_PROTOC_PLUGIN
    ${gz-msgs_PROTO_GENERATOR_PLUGIN}
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

if(ENABLE_GUI)
  set(gui_sources
    ${gui_sources}
    PARENT_SCOPE
  )
endif()

set(material_sources
  rendering/MaterialParser/MaterialParser.cc
  rendering/MaterialParser/ConfigLoader.cc
)

set (sources
  Actor.cc
  AllComponents.cc
  Barrier.cc
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

# Tests that require a valid display
set(tests_needing_display
  Server_Rendering_TEST.cc
)

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
  set_source_files_properties(${sources} ${gtest_sources} COMPILE_FLAGS "/wd4251 /wd4146")
endif()

# Create the library target
gz_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

if(GZ_SIM_ENABLE_PCH)
  target_precompile_headers(${PROJECT_LIBRARY_TARGET_NAME} REUSE_FROM gz-sim-pch)

  # AllComponents.cc defines GZ_SIM_COMPONENT_DEFINITION_TU before its includes
  # to turn GZ_SIM_DECLARE_COMPONENT into full registration. The PCH would
  # force-include Factory.hh ahead of that define, baking the declare-only
  # branch and silently dropping every component registration. Keep this TU off
  # the PCH.
  set_source_files_properties(AllComponents.cc PROPERTIES
    SKIP_PRECOMPILE_HEADERS ON)
endif()

gz_add_get_install_prefix_impl(GET_INSTALL_PREFIX_FUNCTION gz::sim::getInstallPrefix
                               GET_INSTALL_PREFIX_HEADER gz/sim/InstallationDirectories.hh
                               OVERRIDE_INSTALL_PREFIX_ENV_VARIABLE GZ_SIM_INSTALL_PREFIX)

set_property(
  SOURCE InstallationDirectories.cc
  PROPERTY COMPILE_DEFINITIONS
  GZ_SIM_GUI_CONFIG_RELATIVE_PATH="${GZ_DATA_INSTALL_DIR}/gui"
  GZ_SIM_SYSTEM_CONFIG_RELATIVE_PATH="${GZ_DATA_INSTALL_DIR}/systems"
  GZ_SIM_SERVER_CONFIG_RELATIVE_PATH="${GZ_DATA_INSTALL_DIR}"
  GZ_SIM_PLUGIN_RELATIVE_INSTALL_DIR="${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}/plugins"
  GZ_SIM_GUI_PLUGIN_RELATIVE_INSTALL_DIR="${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}/plugins/gui"
  GZ_SIM_WORLD_RELATIVE_INSTALL_DIR="${GZ_DATA_INSTALL_DIR}/worlds"
  GZ_SIM_MEDIA_RELATIVE_INSTALL_DIR="${GZ_DATA_INSTALL_DIR}/media"
)
install(FILES "rendering/MaterialParser/gazebo.material" DESTINATION ${GZ_DATA_INSTALL_DIR}/media)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  gz-math
  gz-plugin::core
  gz-common::gz-common
  gz-common::graphics
  gz-common::profiler
  gz-fuel_tools::gz-fuel_tools
  gz-physics::core
  gz-rendering::core
  gz-transport::gz-transport
  gz-transport::parameters
  sdformat::sdformat
  protobuf::libprotobuf
  PRIVATE
  gz-plugin::loader
)

if(ENABLE_GUI)
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
    PUBLIC
    gz-gui::gz-gui
  )
endif()

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
    gz-sim
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

# Some server unit tests require rendering.
if (TARGET UNIT_Server_Rendering_TEST)
 target_link_libraries(UNIT_Server_Rendering_TEST
    gz-rendering::gz-rendering
  )
endif()

add_subdirectory(cmd)
