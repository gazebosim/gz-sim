set(TEST_TYPE "PERFORMANCE")

set(tests
  each.cc
  level_manager.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

gz_build_tests(TYPE PERFORMANCE
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
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${PROJECT_VERSION_MAJOR}
    gz-sim${PROJECT_VERSION_MAJOR}-gui
)

