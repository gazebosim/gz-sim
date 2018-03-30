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
ign_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  PRIVATE
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ${SDFormat_LIBRARIES})

# Turn on C++14 support for the library
ign_set_project_public_cxx_standard(14)

# Create installation instructions for the library target. This must be called
# in the same scope that the target is created.
ign_install_library()

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# Create the gazebo executable
ign_add_executable(ign-gazebo main.cc)
target_link_libraries(ign-gazebo PUBLIC ${PROJECT_LIBRARY_TARGET_NAME}
    ${IGNITION-GUI_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES}
    gflags)
