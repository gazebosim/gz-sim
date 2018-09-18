set(TEST_TYPE "INTEGRATION")

set(tests
  physics_system.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)
include_directories(${PROJECT_SOURCE_DIR}/test)

ign_build_tests(TYPE INTEGRATION SOURCES ${tests})
