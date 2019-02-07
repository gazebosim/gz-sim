set(TEST_TYPE "INTEGRATION")

set(tests
  altimeter_system.cc
  components.cc
  diff_drive_system.cc
  each_new_removed.cc
  entity_erase.cc
  events.cc
  examples_build.cc
  level_manager.cc
  model.cc
  physics_system.cc
  play_pause.cc
  sensors_system.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)
include_directories(${PROJECT_SOURCE_DIR}/test)

ign_build_tests(TYPE INTEGRATION SOURCES ${tests})
