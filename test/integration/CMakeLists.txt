set(TEST_TYPE "INTEGRATION")

set(tests
  altimeter_system.cc
  components.cc
  diff_drive_system.cc
  each_new_removed.cc
  entity_erase.cc
  events.cc
  examples_build.cc
  imu_system.cc
  level_manager.cc
  magnetometer_system.cc
  model.cc
  network_handshake.cc
  physics_system.cc
  play_pause.cc
  pose_publisher_system.cc
  sdf_include.cc
  user_commands.cc
)

# Tests that require a valid display
set(tests_needing_display
  gpu_lidar.cc
  sensors_system.cc
)

# Add systems that need a valid display here.
# \todo(anyone) Find a way to run these tests with a virtual display such Xvfb
# or Xdummy instead of skipping them
if(VALID_DISPLAY AND VALID_DRI_DISPLAY)
  list(APPEND tests ${tests_needing_display})
else()
  message(STATUS
    "Skipping these INTEGRATION tests because a valid display was not found:")
  foreach(test ${tests_needing_display})
    message(STATUS " ${test}")
  endforeach(test)
endif()

link_directories(${PROJECT_BINARY_DIR}/test)
include_directories(${PROJECT_SOURCE_DIR}/test)

ign_build_tests(TYPE INTEGRATION SOURCES ${tests})
