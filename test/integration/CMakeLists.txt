set(TEST_TYPE "INTEGRATION")

set(tests
  components.cc
  each_new_erased.cc
  entity_erase.cc
  events.cc
  examples_build.cc
  model.cc
  physics_system.cc
  play_pause.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)
include_directories(${PROJECT_SOURCE_DIR}/test)

ign_build_tests(TYPE INTEGRATION SOURCES ${tests})
