set(TEST_TYPE "PERFORMANCE")

set(tests
  each.cc
  level_manager.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(TYPE PERFORMANCE SOURCES ${tests})
