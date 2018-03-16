set (sources
  Entity.cc
  PhysicsSystem.cc
  Server.cc
  System.cc
)

set (gtest_sources
  Entity_TEST.cc
  System_TEST.cc
)

# Create the library target
ign_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  ${SDFormat_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES})

# Turn on C++14 support for the library
ign_set_project_public_cxx_standard(14)

# Create installation instructions for the library target. This must be called
# in the same scope that the target is created.
ign_install_library()

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# Create the gazebo executable
ign_add_executable(gazebo main.cc)
target_link_libraries(gazebo PUBLIC ${PROJECT_LIBRARY_TARGET_NAME} gflags)
