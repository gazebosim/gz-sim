set(TEST_TYPE "INTEGRATION")

set(tests
  entity_erase.cc
  events.cc
  examples_build.cc
  physics_system.cc
  play_pause.cc
  sdf_include.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)
include_directories(${PROJECT_SOURCE_DIR}/test)

ign_build_tests(TYPE INTEGRATION SOURCES ${tests})
