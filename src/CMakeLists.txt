set (sources
  Entity.cc
  TestSystem.cc
  Server.cc
  ServerPrivate.cc
  System.cc
)

set (gtest_sources
  Entity_TEST.cc
  Server_TEST.cc
  System_TEST.cc
)

# Create the library target
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 14)
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
)

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# Create the gazebo executable
ign_add_executable(ign-gazebo main.cc)
target_link_libraries(ign-gazebo PUBLIC ${PROJECT_LIBRARY_TARGET_NAME}
    gflags)

# Switch to ign_install_executable when it is fixed (https://bitbucket.org/ignitionrobotics/ign-cmake/issues/29)
set_target_properties(ign-gazebo PROPERTIES VERSION ${PROJECT_VERSION_FULL})
install (TARGETS ign-gazebo DESTINATION ${IGN_BIN_INSTALL_DIR})
