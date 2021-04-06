set(TEST_TYPE "PERFORMANCE")

set(tests
  each.cc
  level_manager.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(TYPE PERFORMANCE
  SOURCES
    ${tests}
  LIB_DEPS
    ${EXTRA_TEST_LIB_DEPS}
)

add_executable(
  PERFORMANCE_sdf_runner
  sdf_runner.cc
)

target_link_libraries(
  PERFORMANCE_sdf_runner
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-gazebo${PROJECT_VERSION_MAJOR}
    ignition-gazebo${PROJECT_VERSION_MAJOR}-gui
)

